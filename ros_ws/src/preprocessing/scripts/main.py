from custom_msg.msg import Landmarks
from dataclasses import dataclass
from time import time
from typing import List, Optional, Tuple
import bisect
import math
import rospy
import statistics


# Initialize first_timestamp as None
first_timestamp = None
# Calibration period in seconds
CALIBRATION_TIME = 10
# Setup margin period in seconds prior to calibration included in CALIBRATION_TIME
SETUP_MARGIN_TIME = 5

left_arm_length: float = None
right_arm_length: float = None

@dataclass
class Frame:
    """Class to store all data related to a single frame."""
    timestamp: float
    bodypoints: Optional[List[List[float]]] = None
    robot_angles: Optional[List[float]] = None
    robot_bodypoints: Optional[List[Tuple[float, float, float]]] = None

# List of frames ordered by timestamp
frames: List[Frame] = []

pub = rospy.Publisher('preprocessed', Landmarks, queue_size=10)


def process_bodypoints(msg):
    """
    Subscribe to a ROS topic to receive bodypoints.
    Frame data should come in by strictly increasing timestamp.
    The first 10 seconds are given to calibration without any angle pubishing.

    Args:
        msg: ROS message in the form of msg.Landmarks

    Returns:
        None. Angle results are published to ROS.
    """
    global first_timestamp
    global left_arm_length
    global right_arm_length

    begin_ts = int(time() * 1000)
    # Parse msg
    landmarks = msg
    timestamp = landmarks.timestamp

    # Initialize first timestamp if not set
    if first_timestamp is None:
        first_timestamp = timestamp
        rospy.loginfo(f"Starting calibration period of {CALIBRATION_TIME} seconds. Will begin calibrating in {SETUP_MARGIN_TIME} seconds. Please T-Pose.")
    
    # When calibrating we won't publish results, but we'll still grab frames to have past data to work from in the future
    calibrating = False
    time_elapsed = (timestamp - first_timestamp) / 1000  # Convert to seconds
    if time_elapsed < CALIBRATION_TIME:
        calibrating = True

        if time_elapsed > SETUP_MARGIN_TIME:
            # We've given time to get in position
            # We're in the calibration period, run calibration before normal processing (without publishing results)
            _calibrate(msg)

    # If this is the first frame we're done calibrating for lets set our arm length values
    if not calibrating:
        if not left_arm_length:
            left_arm_length = _get_left_arm_length()
            rospy.loginfo(f"Left arm length = {left_arm_length}")
        if not right_arm_length:
            right_arm_length = _get_right_arm_length
            rospy.loginfo(f"Right arm length = {right_arm_length}")

    bodypoints = [
        [x for x in landmarks.nose],
        [x for x in landmarks.left_hip],
        [x for x in landmarks.left_shoulder],
        [x for x in landmarks.left_elbow],
        [x for x in landmarks.left_wrist],
        [x for x in landmarks.left_pinky],
        [x for x in landmarks.left_index],
        [x for x in landmarks.left_thumb],
        [x for x in landmarks.right_hip],
        [x for x in landmarks.right_shoulder],
        [x for x in landmarks.right_elbow],
        [x for x in landmarks.right_wrist],
        [x for x in landmarks.right_pinky],
        [x for x in landmarks.right_index],
        [x for x in landmarks.right_thumb],
    ]
    
    # Create a new frame and add it to our frames list
    current_frame = Frame(timestamp=timestamp, bodypoints=bodypoints)
    frames.append(current_frame)
    current_index = len(frames) - 1
    
    _drop_bad_points(current_index)
    dropped_bad_points_ts = int(time() * 1000)
    
    try:
        _find_missing_points(current_index)
    except ValueError:
        # Skip this frame if it's the first frame and has missing points
        frames.pop()  # Remove the frame we just added
        return
    found_missing_points_ts = int(time() * 1000)
    
#    _smooth_points(current_index)
    _smooth_points(current_index)
    smoothed_points_ts = int(time() * 1000)

    _translate_points(current_index)

    
    # Calculate robot angles based on the (now processed) bodypoints
    #_get_robotangles(current_index)
    #_restrain_angles(current_index)

    #_restrain_position(current_index)
    #_restrain_speed(current_index)
    
    #_get_robotangles_from_robot_bodypoints(current_index)

    preprocessed_msg = Landmarks()
    preprocessed_msg.nose           = current_frame.bodypoints[0]
    preprocessed_msg.left_hip       = current_frame.bodypoints[1]
    preprocessed_msg.left_shoulder  = current_frame.bodypoints[2]
    preprocessed_msg.left_elbow     = current_frame.bodypoints[3]
    preprocessed_msg.left_wrist     = current_frame.bodypoints[4]
    preprocessed_msg.left_pinky     = current_frame.bodypoints[5]
    preprocessed_msg.left_index     = current_frame.bodypoints[6]
    preprocessed_msg.left_thumb     = current_frame.bodypoints[7]
    preprocessed_msg.right_hip      = current_frame.bodypoints[8]
    preprocessed_msg.right_shoulder = current_frame.bodypoints[9]
    preprocessed_msg.right_elbow    = current_frame.bodypoints[10]
    preprocessed_msg.right_wrist    = current_frame.bodypoints[11]
    preprocessed_msg.right_pinky    = current_frame.bodypoints[12]
    preprocessed_msg.right_index    = current_frame.bodypoints[13]
    preprocessed_msg.right_thumb    = current_frame.bodypoints[14]
    preprocessed_msg.timestamp      = current_frame.timestamp

    if not calibrating:
        # We don't want to publish if we're calibrating still
        pub.publish(preprocessed_msg)
        published_ts = int(time() * 1000)
        rospy.loginfo(f"({published_ts - begin_ts}ms) {preprocessed_msg.right_wrist}")
    
    # publish robot angles
    #angles_msg = Angles()
    #current_frame = frames[current_index]
    #angles_msg.left_arm = current_frame.robot_angles[0]
    #angles_msg.right_arm = current_frame.robot_angles[1]
    #pub.publish(angles_msg)


# A sorted list of calculated arm lengths from frames
_left_arm_lengths: List[float] = []
_right_arm_lengths: List[float] = []


def _get_left_arm_length() -> float:
    """
    Get the maximum found left arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    return _get_arm_length(_left_arm_lengths)


def _get_right_arm_length() -> float:
    """
    Get the maximum found right arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    return _get_arm_length(_right_arm_lengths)


def _get_arm_length(sorted_lengths: List[float]) -> float:
    """
    Get the maximum found arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    if not sorted_lengths:
        return 0.0

    if len(sorted_lengths) < 4:
        # If not enough data to meaningfully detect outliers, return max
        return sorted_lengths[-1]

    # Sort the lengths to compute quartiles
    q1 = statistics.quantiles(sorted_lengths, n=4)[0]  # 25th percentile
    q3 = statistics.quantiles(sorted_lengths, n=4)[2]  # 75th percentile
    iqr = q3 - q1

    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr

    # Filter out outliers
    filtered = [l for l in sorted_lengths if lower_bound <= l <= upper_bound]

    if not filtered:
        return sorted_lengths[-1]  # fallback if everything was removed

    return sorted_lengths[-1]


def _calibrate(msg):
    """
    Perform calibration to obtain arm lengths.
    
    Args:
        bodypoints: The bodypoints from the current frame
        timestamp: The timestamp of the current frame
        
    Returns:
        None. Calibration data is stored internally.
    """

    left_points = [msg.left_shoulder, msg.left_elbow, msg.left_wrist]
    right_points = [msg.right_shoulder, msg.right_elbow, msg.right_wrist]

    # Check if any points do not exist or have invalid values
    for points in [left_points, right_points]:
        for point in points:
            # Check if point is None, empty, or contains None values
            if point is None or len(point) < 3 or None in point:
                return
            
            # Also check for zero values in all coordinates (likely invalid data)
            if all(v == 0 for v in point):
                return

    left_length = _get_length(left_points)
    right_length = _get_length(right_points)

    bisect.insort(_left_arm_lengths, left_length)
    bisect.insort(_right_arm_lengths, right_length)


def _get_length(points: List[List[float]]) -> float:
    """
    Get the length of the ordered set of 3D points.
    """

    if len(points) < 2:
        return 0.0

    total_length = 0.0
    for i in range(1, len(points)):
        p1, p2 = points[i - 1], points[i]
        distance = math.sqrt(
            (p2[0] - p1[0]) ** 2 +
            (p2[1] - p1[1]) ** 2 +
            (p2[2] - p1[2]) ** 2
        )
        total_length += distance

    return total_length


# TODO: if points look unstable, try dropping low-confidence mediapipe points to be replaced via interpolation
def _drop_bad_points(frame_index: int):
    """
    Drop any points at the specified frame index whose values are incorrect.
    
    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The bodypoints are updated in-place in the frames list
    """
    
    current_frame = frames[frame_index]
    bodypoints = current_frame.bodypoints
    
    # Filter out any points with z-value = 0 (depth camera failure)
    for point in bodypoints:
        if point is not None and point[2] == 0:
            point[2] = None
    
    # Update the frame with filtered points
    current_frame.bodypoints = bodypoints


# TODO: if interpolation is bad, make it smarter by using points from the future too (where they are present)
def _find_missing_points(frame_index: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints.
    If this is the first frame and we have missing points, throws an exception so the processing
    loop can skip this frame and wait for a better first frame.

    Uses velocity-based extrapolation from previous frames to predict the position of missing points.
    We guarantee that after this function is called, all bodypoints in the frame will be populated.
    
    Args:
        frame_index: Index of the frame in the frames list

    Raises:
        ValueError: If this is the first frame and it has missing points

    Returns:
        None. The bodypoints are updated in-place in the frames list
    """
    
    current_frame = frames[frame_index]
    current_points = current_frame.bodypoints
    
    # Check if any points are missing entirely or have missing coordinates
    has_missing_data = False
    for point in current_points:
        if point is None:
            has_missing_data = True
            break
        if len(point) >= 3 and (point[0] is None or point[1] is None or point[2] is None):
            has_missing_data = True
            break
    
    # If this is the first frame
    if frame_index == 0:
        if has_missing_data:
            raise ValueError("First frame has missing points - cannot interpolate without previous data")
        return  # First frame is complete, nothing to do
    
    # Look for the previous frames to calculate velocity
    # We will average multiple frames to smooth values
    NUM_HISTORY_FRAMES = 5  # Number of previous frames to use for velocity calculation
    available_indices = []
    
    # Find available previous frames, starting from most recent
    # It is guaranteed that there will be at least one
    for i in range(1, min(NUM_HISTORY_FRAMES + 1, frame_index + 1)):
        available_indices.append(frame_index - i)
    #rospy.loginfo("Available indices for extrapolating: " + str(available_indices))

    # Get the most recent frame's points
    prev_frame = frames[available_indices[0]]
    prev_points = prev_frame.bodypoints
    
    # For each point in the current frame
    for i in range(len(current_points)):
        # If point is completely missing, we need to predict all coordinates
        if current_points[i] is None:
            #rospy.loginfo("point " + str(i) + " is totally missing")
            current_points[i] = (None, None, None)
        
        # Get current coordinates (which may be None)
        curr_x = current_points[i][0]
        curr_y = current_points[i][1]
        curr_z = current_points[i][2]
        #rospy.loginfo(f"curr_xyz: ({curr_x}, {curr_y}, {curr_z})")
        
        # Determine which coordinates need prediction
        needs_x = curr_x is None
        needs_y = curr_y is None
        needs_z = curr_z is None
        
        # If nothing needs prediction, skip this point
        if not (needs_x or needs_y or needs_z):
            #rospy.loginfo("No prediction on this point!")
            continue
        
        # If we only have one previous frame, use its coordinates for missing values
        if len(available_indices) == 1:
            prev_x, prev_y, prev_z = prev_points[i]
            new_x = prev_x if needs_x else curr_x
            new_y = prev_y if needs_y else curr_y
            new_z = prev_z if needs_z else curr_z
            current_points[i] = (new_x, new_y, new_z)
        else:
            #rospy.loginfo("Multiple frames, calculate velocities... ")
            # We have multiple frames, calculate velocities for needed coordinates
            vx_values = []
            vy_values = []
            vz_values = []
            
            # Calculate velocities between consecutive frames
            for j in range(len(available_indices) - 1):
                #rospy.loginfo(f"iterating for vel: {j}; curr_idx={available_indices[j]}; prev_idx={available_indices[j+1]}")
                curr_idx = available_indices[j]
                prev_idx = available_indices[j + 1]
                
                curr_frame_pts = frames[curr_idx].bodypoints
                prev_frame_pts = frames[prev_idx].bodypoints
                
                curr_point = curr_frame_pts[i]
                prev_point = prev_frame_pts[i]
                #rospy.loginfo(f"time diff: {frames[curr_idx].timestamp} - {frames[prev_idx].timestamp}")
                # Calculate time difference
                time_diff = frames[curr_idx].timestamp - frames[prev_idx].timestamp
                
                if needs_x:
                    vx = (curr_point[0] - prev_point[0]) / time_diff
                    vx_values.append(vx)
                    
                if needs_y:
                    vy = (curr_point[1] - prev_point[1]) / time_diff
                    vy_values.append(vy)
                    
                if needs_z:
                    vz = (curr_point[2] - prev_point[2]) / time_diff
                    vz_values.append(vz)
            
            # Get most recent point and time since last frame
            most_recent_point = prev_points[i]
            time_since_last = current_frame.timestamp - prev_frame.timestamp
            
            # Calculate new coordinates only for those that need prediction
            new_x = curr_x
            new_y = curr_y
            new_z = curr_z
            
            if needs_x:
                avg_vx = sum(vx_values) / len(vx_values)
                new_x = most_recent_point[0] + avg_vx * time_since_last
                
            if needs_y:
                avg_vy = sum(vy_values) / len(vy_values)
                new_y = most_recent_point[1] + avg_vy * time_since_last
                
            if needs_z:
                avg_vz = sum(vz_values) / len(vz_values)
                new_z = most_recent_point[2] + avg_vz * time_since_last
            
            # Update only the coordinates that needed prediction
            current_points[i] = (new_x, new_y, new_z)
    
    # Update the frame with our interpolated points
    current_frame.bodypoints = current_points


def _smooth_points(frame_index: int):
    """
    Smoothen points at the specified frame by using surrounding frames, reducing noise and jitter.
    Ignores small variations between consecutive frames to produce smoother motion.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The bodypoints are updated in-place in the frames list
    """
    
    # If this is the first frame, no smoothing needed
    if frame_index == 0:
        return
    
    current_frame = frames[frame_index]
    prev_frame = frames[frame_index - 1]
    
    current_points = current_frame.bodypoints
    prev_points = prev_frame.bodypoints
    
    # Define threshold for jitter (adjust as needed for sensitivity)
    JITTER_THRESHOLD = 0.01  # Small movement threshold
    
    # For each point in the current frame
    for i in range(len(current_points)):
        # Skip if either point is None (shouldn't happen after _find_missing_points)
        if current_points[i] is None or prev_points[i] is None:
            #rospy.logerr(f"current_points[i] or prev_points[i] is None! {i}")
            continue
        
        # Unpack the current and previous 3D coordinates
        # Each point is (x, y, z)
        curr_x, curr_y, curr_z = current_points[i]
        prev_x, prev_y, prev_z = prev_points[i]
        #rospy.loginfo(f"curr_xyz: ({curr_x}, {curr_y}, {curr_z}), {prev_points}")
        # Calculate 3D Euclidean distance between current and previous point
        dx = curr_x - prev_x
        dy = curr_y - prev_y
        dz = curr_z - prev_z
        movement_distance = (dx**2 + dy**2 + dz**2)**0.5
        
        # If movement is small (jitter), use the previous frame's position
        if movement_distance < JITTER_THRESHOLD:
            current_points[i] = (prev_x, prev_y, prev_z)
    
    # Update the frame with smoothed points
    current_frame.bodypoints = current_points

def _translate_points(frame_index : int):
    """
    Convert all bodypoints at the given frame to relative to the midpoint of the hips
    by averaging the midpoint of the hips and subtracting it from each point entrywise.

    Assumes all points at the given frame are valid.
    """
    current_frame = frames[frame_index]
    left_hip = current_frame.bodypoints[1]
    right_hip = current_frame.bodypoints[8]
    hip_middle = [
        (left_hip[0] + right_hip[0]) / 2,
        (left_hip[1] + right_hip[1]) / 2,
        (left_hip[2] + right_hip[2]) / 2,
    ]

    relative_bodypoints = []
    
    for point in current_frame.bodypoints:
        relative_bodypoints.append([
            ((point[0] - hip_middle[0])),
            ((point[1] - hip_middle[1])),
            point[2] - hip_middle[2]
        ])
    current_frame.bodypoints = relative_bodypoints
    


def _get_robotangles(frame_index: int):
    """
    Convert human bodypoints to corresponding robot joint angles.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The robot angles are updated in-place in the frame
    """

    pass



def _restrain_angles(frame_index: int):
    """
    Apply angle constraints to ensure the robot stays within its physical limits.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The robot angles are updated in-place in the frame
    """

    pass


def _restrain_position(frame_index: int):
    """
    Apply position constraints to prevent self-collision and cord tangling.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. Results are stored in the frame's robot_bodypoints
    """

    pass


def _restrain_speed(frame_index: int):
    """
    Apply speed constraints to ensure the robot's movements stay within velocity limits.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The robot bodypoints are updated in-place in the frame
    """

    pass


def _get_robotangles_from_robot_bodypoints(frame_index: int):
    """
    Convert robot bodypoints (after constraints are applied) back to robot joint angles.

    Args:
        frame_index: Index of the frame in the frames list

    Returns:
        None. The robot angles are updated in-place in the frame
    """

    pass

def pubtest():
    t_pose_relative_to_hips = Landmarks()
    t_pose_relative_to_hips.left_hip       = [-45,0,0]
    t_pose_relative_to_hips.left_wrist     = [183, 95, 0]
    t_pose_relative_to_hips.right_hip      = [45,0,0]
    t_pose_relative_to_hips.right_shoulder = [45,95,0]
    t_pose_relative_to_hips.right_wrist    = [-183, 95, 0]
    t_pose_relative_to_hips.timestamp      = int(time() * 1000)


    t_pose_absolute = Landmarks()
    t_pose_absolute.left_hip            = [-0.125,0,0]
    t_pose_absolute.left_wrist          = [.508, .27, 0]
    t_pose_absolute.left_shoulder       = [-0.125,.27,0]
    t_pose_absolute.right_hip           = [.125,0,0]
    t_pose_absolute.right_wrist         = [-.508, .27, 0]
    t_pose_absolute.right_shoulder      = [.125,.27,0]


    a_pose_absolute = Landmarks()
    a_pose_absolute.left_hip            = [-0.125,0,0]
    a_pose_absolute.left_shoulder       = [-0.125,.27,0]
    a_pose_absolute.left_wrist          = [.25, 0, 0]
    a_pose_absolute.right_hip           = [.125,0,0]
    a_pose_absolute.right_shoulder      = [.125,.27,0]
    a_pose_absolute.right_wrist         = [-.25, 0, 0]

    pub.publish(t_pose_absolute)


def app():
    rospy.Subscriber('landmarks', Landmarks, process_bodypoints)
    rospy.init_node('preprocessing', anonymous=True)
    #pubtest()
    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

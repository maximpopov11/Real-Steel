from custom_msg.msg import Landmarks, Angles
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy


# TODO: should this file live elsewhere now?


@dataclass
class Frame:
    """Class to store all data related to a single frame."""
    timestamp: float
    bodypoints: Optional[List[Tuple[float, float, float]]] = None
    robot_angles: Optional[List[float]] = None
    robot_bodypoints: Optional[List[Tuple[float, float, float]]] = None

# List of frames ordered by timestamp
frames: List[Frame] = []

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)


def process_bodypoints(msg):
    """
    Subscribe to a ROS topic to receive bodypoints.
    Frame data should come in by strictly increasing timestamp.

    Args:
        msg: ROS message in the form of msg.Landmarks

    Returns:
        None. Angle results are published to ROS.
    """

    # Parse msg
    landmarks = msg.Landmarks
    timestamp = landmarks.timestamp
    bodypoints = [
        landmarks.nose,
        landmarks.left_hip,
        landmarks.left_shoulder,
        landmarks.left_elbow,
        landmarks.left_wrist,
        landmarks.left_pinky,
        landmarks.left_index,
        landmarks.left_thumb,
        landmarks.right_hip,
        landmarks.right_shoulder,
        landmarks.right_elbow,
        landmarks.right_wrist,
        landmarks.right_pinky,
        landmarks.right_index,
        landmarks.right_thumb,
    ]
    
    # Create a new frame and add it to our frames list
    current_frame = Frame(timestamp=timestamp, bodypoints=bodypoints)
    frames.append(current_frame)
    current_index = len(frames) - 1
    
    _drop_bad_points(current_index)
    
    try:
        _find_missing_points(current_index)
    except ValueError:
        # Skip this frame if it's the first frame and has missing points
        frames.pop()  # Remove the frame we just added
        return
        
    _smooth_points(current_index)
    
    # Calculate robot angles based on the (now processed) bodypoints
    _get_robotangles(current_index)
    _restrain_angles(current_index)

    _restrain_position(current_index)
    _restrain_speed(current_index)
    
    _get_robotangles_from_robot_bodypoints(current_index)

    # publish robot angles
    angles_msg = Angles()
    current_frame = frames[current_index]
    angles_msg.left_arm = current_frame.robot_angles[0]
    angles_msg.right_arm = current_frame.robot_angles[1]
    pub.publish(angles_msg)


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


    # Get the most recent frame's points
    prev_frame = frames[available_indices[0]]
    prev_points = prev_frame.bodypoints
    
    # For each point in the current frame
    for i in range(len(current_points)):
        # If point is completely missing, we need to predict all coordinates
        if current_points[i] is None:
            current_points[i] = (None, None, None)
        
        # Get current coordinates (which may be None)
        curr_x = current_points[i][0]
        curr_y = current_points[i][1]
        curr_z = current_points[i][2]
        
        # Determine which coordinates need prediction
        needs_x = curr_x is None
        needs_y = curr_y is None
        needs_z = curr_z is None
        
        # If nothing needs prediction, skip this point
        if not (needs_x or needs_y or needs_z):
            continue
        
        # If we only have one previous frame, use its coordinates for missing values
        if len(available_indices) == 1:
            prev_x, prev_y, prev_z = prev_points[i]
            new_x = prev_x if needs_x else curr_x
            new_y = prev_y if needs_y else curr_y
            new_z = prev_z if needs_z else curr_z
            current_points[i] = (new_x, new_y, new_z)
        else:
            # We have multiple frames, calculate velocities for needed coordinates
            vx_values = []
            vy_values = []
            vz_values = []
            
            # Calculate velocities between consecutive frames
            for j in range(len(available_indices) - 1):
                curr_idx = available_indices[j]
                prev_idx = available_indices[j + 1]
                
                curr_frame_pts = frames[curr_idx].bodypoints
                prev_frame_pts = frames[prev_idx].bodypoints
                
                curr_point = curr_frame_pts[i]
                prev_point = prev_frame_pts[i]
                
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
            continue
        
        # Unpack the current and previous 3D coordinates
        # Each point is (x, y, z)
        curr_x, curr_y, curr_z = current_points[i]
        prev_x, prev_y, prev_z = prev_points[i]
        
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


def app():
    rospy.Subscriber('landmarks', Landmarks, process_bodypoints)
    rospy.init_node('kinematics', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

from custom_types import Bodypoints_t, Robot_Angles_t
from queue import PriorityQueue
from typing import Dict
import threading


# TODO: plug into ROS: look at how kinematics does it, custom_msg package has our bodypoints (I am the framework then nice!)
# TODO: don't assume timestamps are 1, 2, 3, ...
# TODO: don't assume timestamps will come in order
# TODO: make processing smarter by using points in the future too
def process_bodypoints(
    timestamps: PriorityQueue[int],
    bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    robot_angles_by_timestamp: Dict[int, Robot_Angles_t],
):
    """
    Spawn a thread to process Bodypoints and load them into the RobotAngles queue.
    Processing happens in the background through a daemon thread.

    Args:
        timestamps: Priority queue of timestamps to process, in chronological order where timestamps encompass all natural nums 1, 2, ...
        bodypoints_by_timestamp: Dictionary mapping timestamps to their corresponding bodypoints
        robot_angles_by_timestamp: Dictionary to store the processed robot angles for each timestamp

    Returns:
        None. Processing results are stored in robot_angles_by_timestamp
    """

    processing_thread = threading.Thread(
        target=_process_bodypoints_loop,
        args=(timestamps, bodypoints_by_timestamp, robot_angles_by_timestamp),
        daemon=True  # Make thread daemon so it exits when main program exits
    )
    processing_thread.start()


def _process_bodypoints_loop(
    timestamps: PriorityQueue[int],
    bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    robot_angles_by_timestamp: Dict[int, Robot_Angles_t],
):
    """
    Main processing loop that runs in a separate thread. Continuously processes incoming
    bodypoints and converts them to robot angles.

    Args:
        timestamps: Priority queue of timestamps to process, in chronological order
        bodypoints_by_timestamp: Dictionary mapping timestamps to their corresponding bodypoints
        robot_angles_by_timestamp: Dictionary to store the processed robot angles for each timestamp

    Returns:
        None. Processing results are stored in robot_angles_by_timestamp
    """

    robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t] = {}

    while True:
        timestamp = timestamps.get()  # Blocks efficiently until data is available

        _drop_bad_points(bodypoints_by_timestamp, timestamp)
        
        try:
            _find_missing_points(bodypoints_by_timestamp, timestamp)
        except ValueError:
            # Skip this frame if it's the first frame and has missing points
            continue
            
        _smooth_points(bodypoints_by_timestamp, timestamp)
        bodypoints = bodypoints_by_timestamp[timestamp]

        robot_angles = _get_robotangles(bodypoints)
        restrained_angles = _restrain_angles(robot_angles)

        _restrain_position(restrained_angles, robot_bodypoints_by_timestamp, timestamp)
        _restrain_speed(robot_bodypoints_by_timestamp, timestamp)
        
        robot_bodypoints = robot_bodypoints_by_timestamp[timestamp]
        final_angles = _get_robotangles_from_robot_bodypoints(robot_bodypoints)
        robot_angles_by_timestamp[timestamp] = final_angles


def _drop_bad_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Drop any points at the timstamp who's values are incorrect.
    """
    bodypoints = bodypoints_by_timestamp[timestamp]
    
    # Filter out any points with z-value = 0 (depth camera failure)
    for point in bodypoints.items():
        if point is not None and point[2] == 0:
            point[2] = None
    
    # Update the dictionary with filtered points
    bodypoints_by_timestamp[timestamp] = bodypoints


# TODO: our z-values may have been dropped
def _find_missing_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints at the timestamp.
    If this is the first frame and we have missing points, throws an exception so the processing
    loop can skip this frame and wait for a better first frame.

    Uses velocity-based extrapolation from previous frames to predict the position of missing points.
    We guarantee that after this function is called, all bodypoints in the timestamp will be populated.
    
    Args:
        bodypoints_by_timestamp: Dictionary mapping timestamps to their corresponding bodypoints
        timestamp: The timestamp of the frame to process

    Raises:
        ValueError: If this is the first frame and it has missing points

    Returns:
        None. The bodypoints are updated in-place in the bodypoints_by_timestamp dictionary
    """

    current_points = bodypoints_by_timestamp[timestamp]
    
    # Check if any points are missing entirely or have missing coordinates
    has_missing_data = False
    for point in current_points:
        if point is None:
            has_missing_data = True
            break
        if len(point) >= 3 and (point[0] is None or point[1] is None or point[2] is None):
            has_missing_data = True
            break
    
    if timestamp - 1 not in bodypoints_by_timestamp:
        # If this is the first frame (no previous frame exists)
        if has_missing_data:
            raise ValueError("First frame has missing points - cannot interpolate without previous data")
        return  # First frame is complete, nothing to do
    
    # Look for the previous frames to calculate velocity
    # We will average multiple frames to smooth values
    NUM_HISTORY_FRAMES = 5  # Number of previous frames to use for velocity calculation
    available_frames = []
    
    # Find available previous frames, already in order from most recent to oldest
    for i in range(1, NUM_HISTORY_FRAMES + 1):
        if timestamp - i in bodypoints_by_timestamp:
            available_frames.append(timestamp - i)
    
    # Need at least one previous frame
    if not available_frames:
        raise ValueError("No previous frames available for interpolation")
    
    # Get the most recent frame's points
    prev_points = bodypoints_by_timestamp[available_frames[0]]
    
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
        if len(available_frames) == 1:
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
            for j in range(len(available_frames) - 1):
                curr_frame = available_frames[j]
                prev_frame = available_frames[j + 1]
                
                curr_point = bodypoints_by_timestamp[curr_frame][i]
                prev_point = bodypoints_by_timestamp[prev_frame][i]
                
                # Calculate velocity components only for needed coordinates
                time_diff = curr_frame - prev_frame  # Assuming uniform timesteps
                
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
            time_since_last = timestamp - available_frames[0]
            
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
    
    # Update the dictionary with our interpolated points
    bodypoints_by_timestamp[timestamp] = current_points


# TODO: how do we normally see jitter? If we have individual points jittering harshly while the rest is constant we can ignore those too
def _smooth_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Smoothen points at the timestamp by using surrounding frames, reducing noise and jitter.
    Ignores small variations between consecutive frames to produce smoother motion.

    Args:
        bodypoints_by_timestamp: Dictionary mapping timestamps to their corresponding bodypoints
        timestamp: The timestamp of the frame to smooth

    Returns:
        None. The bodypoints are updated in-place in the bodypoints_by_timestamp dictionary
    """

    # If this is the first frame, no smoothing needed
    if timestamp - 1 not in bodypoints_by_timestamp:
        return
    
    current_points = bodypoints_by_timestamp[timestamp]
    prev_points = bodypoints_by_timestamp[timestamp - 1]
    
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
    
    # Update the dictionary with smoothed points
    bodypoints_by_timestamp[timestamp] = current_points


def _get_robotangles(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Convert human bodypoints to corresponding robot joint angles.

    Args:
        bodypoints: Array of 33 body keypoints representing the human pose

    Returns:
        Robot_Angles_t: The calculated robot joint angles that would mimic the human pose
    """

    pass


def _get_robotangles_from_robot_bodypoints(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Convert robot bodypoints (after constraints are applied) back to robot joint angles.

    Args:
        bodypoints: Array of robot body keypoints after constraints have been applied

    Returns:
        Robot_Angles_t: The final robot joint angles that satisfy all constraints
    """

    pass


def _restrain_angles(robot_angles: Robot_Angles_t) -> Robot_Angles_t:
    """
    Apply angle constraints to ensure the robot stays within its physical limits.

    Args:
        robot_angles: The initial robot joint angles

    Returns:
        Robot_Angles_t: The robot joint angles after applying angle constraints
    """

    pass


def _restrain_position(
    robot_angles: Robot_Angles_t, 
    robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    timestamp: int
):
    """
    Apply position constraints to prevent self-collision and cord tangling.

    Args:
        robot_angles: The current robot joint angles
        robot_bodypoints_by_timestamp: Dictionary storing robot bodypoints for each timestamp
        timestamp: The current timestamp being processed

    Returns:
        None. Results are stored in robot_bodypoints_by_timestamp
    """

    # TODO: calculate robot bodypoints (or whatever type it'll be, we don't need all 33 points), then restrain them, and use them in restrain_speed too
    pass


def _restrain_speed(robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Apply speed constraints to ensure the robot's movements stay within velocity limits.

    Args:
        robot_bodypoints_by_timestamp: Dictionary storing robot bodypoints for each timestamp
        timestamp: The current timestamp being processed

    Returns:
        None. The robot bodypoints are updated in-place in robot_bodypoints_by_timestamp
    """

    pass

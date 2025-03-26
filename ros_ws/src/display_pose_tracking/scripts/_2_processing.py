from custom_types import Bodypoints_t, Robot_Angles_t
from queue import PriorityQueue
from typing import Dict
import threading


# TODO: explain args and returns for all funcs, document good
# TODO: plug into framework file, signatures changed
def process_bodypoints(
    timestamps: PriorityQueue[int],
    bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    robot_angles_by_timestamp: Dict[int, Robot_Angles_t],
):
    """
    Spawn a thread to process Bodypoints and load them into the RobotAngles queue.
    Processing happens in the background through a daemon thread.
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
    Main processing loop that runs in a separate thread.
    """
    robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t] = {}

    while True:
        timestamp = timestamps.get()  # Blocks efficiently until data is available
        
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


# TODO: make this smart by taking velocity into account
# TODO: make this smart by using points in the future too
def _find_missing_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints at the timestamp.
    If this is the first frame and we have missing points, throws an exception so the processing
    loop can skip this frame and wait for a better first frame.

    We guarantee that after this function is called, all bodypoints in the timestamp will be populated.
    
    Args:
        bodypoints_by_timestamp: Dictionary mapping timestamps to bodypoints
        timestamp: Current timestamp to process
        
    Raises:
        ValueError: If this is the first frame and we have missing points
    """
    current_points = bodypoints_by_timestamp[timestamp]
    
    has_missing_points = any(point is None for point in current_points)
    if timestamp - 1 not in bodypoints_by_timestamp:
        # If this is the first frame (no previous frame exists)
        if has_missing_points:
            raise ValueError("First frame has missing points - cannot interpolate without previous data")
        return  # First frame is complete, nothing to do
    
    # If we get here, we have a previous frame (which is guaranteed to be complete)
    prev_points = bodypoints_by_timestamp[timestamp - 1]
    
    # Any missing points are set to what they were in the last frame
    for i in range(len(current_points)):
        if current_points[i] is None:
            current_points[i] = prev_points[i]
    
    # Update the dictionary with our interpolated points
    bodypoints_by_timestamp[timestamp] = current_points


# TODO: make this smart by using points in the future too
def _smooth_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to smoothen points at the timestamp, ignoring variations within a small margin.
    """
    # TODO: implement
    pass


def _get_robotangles(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def _get_robotangles_from_robot_bodypoints(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given robot bodypoints to robot angles.
    """
    pass


def _restrain_angles(robot_angles: Robot_Angles_t) -> Robot_Angles_t:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def _restrain_position(
    robot_angles: Robot_Angles_t, 
    robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    timestamp: int
):
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    This will use the given robot_angles to calculate the robot's bodypoints and restrain them as necessary.
    """
    # TODO: calculate robot bodypoints (or whatever type it'll be, we don't need all 33 points), then restrain them, and use them in restrain_speed too
    pass


def _restrain_speed(robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Restrain the robot bodypoints at the given timestamp to disallow moving faster than the threshold.
    """
    pass

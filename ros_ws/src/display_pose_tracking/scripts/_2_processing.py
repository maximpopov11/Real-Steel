from custom_types import Bodypoints_t, Robot_Angles_t
from queue import PriorityQueue
from typing import Dict
import threading


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
        
        _find_missing_points(bodypoints_by_timestamp, timestamp)
        _smooth_points(bodypoints_by_timestamp, timestamp)
        bodypoints = bodypoints_by_timestamp[timestamp]

        robot_angles = _get_robotangles(bodypoints)
        restrained_angles = _restrain_angles(robot_angles)

        _restrain_position(restrained_angles, robot_bodypoints_by_timestamp, timestamp)
        _restrain_speed(robot_bodypoints_by_timestamp, timestamp)
        
        robot_bodypoints = robot_bodypoints_by_timestamp[timestamp]
        final_angles = _get_robotangles_from_robot_bodypoints(robot_bodypoints)
        robot_angles_by_timestamp[timestamp] = final_angles


def _find_missing_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints at the timestamp.
    """
    # TODO: implement
    pass


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
    # TODO: calculate robot bodypoints (or type, we don't need all 33 points), then restrain them, and use them in restrain_speed too
    pass


def _restrain_speed(robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Restrain the robot bodypoints at the given timestamp to disallow moving faster than the threshold.
    """
    pass

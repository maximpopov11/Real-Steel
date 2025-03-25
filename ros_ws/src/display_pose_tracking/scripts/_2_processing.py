from custom_types import Bodypoints_t, Robot_Angles_t
from queue import PriorityQueue
from typing import Dict


# TODO: add placeholders for all funcs to be able to run through this all and output dummy data for what's not done
# TODO: plug into framework file, signatures changed
def process_bodypoints(
    timestamps: PriorityQueue[int],
    bodypoints_by_timestamp: Dict[int, Bodypoints_t],
    robot_angles_by_timestamp: Dict[int, Robot_Angles_t],
):
    """
    Spawn threads to process Bodypoints and load them into the RobotAngles queue.
    """
    # TODO: this is a placeholder for whatever robot bodypoints will actually be
    robot_bodypoints_by_timestamp: Dict[int, Bodypoints_t] = {}

    # TODO: thread max_parallelism at a time, wait if all done until more come in
    # TODO: are we ever looking to the future for things we might have already calculated? If so, do they ever wait and look at the past? How much complexity right now?
    timestamp = timestamps.get()
    
    find_missing_points(bodypoints_by_timestamp, timestamp)
    smooth_points(bodypoints_by_timestamp, timestamp)
    bodypoints = bodypoints_by_timestamp[timestamp]

    robot_angles = get_robotangles(bodypoints)
    restrained_angles = restrain_angles(robot_angles)

    robot_bodypoints = restrain_position(restrained_angles)
    robot_bodypoints_by_timestamp[timestamp] = robot_bodypoints

    final_bodypoints = restrain_speed(robot_bodypoints_by_timestamp, timestamp)
    robot_bodypoints_by_timestamp[timestamp] = final_bodypoints

    final_angles = get_robotangles_from_robot_bodypoints(final_bodypoints)
    robot_angles_by_timestamp[timestamp] = final_angles


# TODO: these aren't all working in a consistent manner, let's be consistent
def find_missing_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints at the timestamp.
    """
    pass


def smooth_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to smoothen points at the timestamp, ignoring variations within a small margin.
    """
    pass


def get_robotangles(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def get_robotangles_from_robot_bodypoints(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given robot bodypoints to robot angles.
    """
    pass


def restrain_angles(robot_angles: Robot_Angles_t) -> Robot_Angles_t:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def restrain_position(robot_angles: Robot_Angles_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    This will use the given robot_angles to calculate the robot's bodypoints and restrain them as necessary.
    """
    # TODO: calculate robot bodypoints (or type, we don't need all 33 points), then restrain them, and use them in restrain_speed too
    pass


def restrain_speed(robot_bodypoints_by_timestamp: dict[int, Bodypoints_t], timestamp: int) -> Bodypoints_t:
    """
    Restrain the robot bodypoints at the given timestamp to disallow moving faster than the threshold.
    """
    pass

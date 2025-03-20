from typing import Dict
from model import Bodypoints_t, RobotAngles_t
from custom_types import ts_Bodypoints_t, q_Robot_Angles_t

def process_bodypoints(
    bodypoints_by_timestamp: ts_Bodypoints_t,
    robot_angles_queue: q_Robot_Angles_t,
):
    """
    Spawn threads to process Bodypoints and load them into the RobotAngles queue.
    """
    pass


def find_missing_points(
    bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int
) -> Bodypoints_t:
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints.
    """
    pass


def smooth_points(bodypoint_queue: Dict[int, Bodypoints_t], index: int) -> Bodypoints_t:
    """
    Use points in surrounding frames to smoothen points, ignoring differences within a small margin.
    """
    pass


def get_robotangles(bodypoints: Bodypoints_t) -> RobotAngles_t:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def restrain_angles(robot_angles: RobotAngles_t) -> RobotAngles_t:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def restrain_position(bodypoints: Bodypoints_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    """
    pass


def restrain_speed(former_bodypoints: Bodypoints_t, bodypoints: Bodypoints_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow moving faster than the threshold.
    """
    pass

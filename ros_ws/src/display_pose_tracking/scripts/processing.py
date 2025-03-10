from typing import Dict
from model import Bodypoints, RobotAngles


def process_bodypoints(
    bodypoint_queue: Dict[int, Bodypoints], robot_angles: Dict[int, RobotAngles]
):
    """
    Spawn threads to process Bodypoints and convert them into RobotAngles.
    """
    pass


def find_missing_points(
    bodypoint_queue: Dict[int, Bodypoints], index: int
) -> Bodypoints:
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints.
    """
    pass


def smooth_points(bodypoint_queue: Dict[int, Bodypoints], index: int) -> Bodypoints:
    """
    Use points in surrounding frames to smoothen points, ignoring differences within a small margin.
    """
    pass


def get_robotangles(bodypoints: Bodypoints) -> RobotAngles:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def restrain_angles(robot_angles: RobotAngles) -> RobotAngles:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def restrain_position(bodypoints: Bodypoints) -> Bodypoints:
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    """
    pass


def restrain_speed(former_bodypoints: Bodypoints, bodypoints: Bodypoints) -> Bodypoints:
    """
    Restrain the given bodypoints to disallow moving faster than the threshold.
    """
    pass

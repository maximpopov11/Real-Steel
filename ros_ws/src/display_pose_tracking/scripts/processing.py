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
    Using the old_points, smooth these new_points.
    """
    pass


def check_feasability(points: arm_markers) -> bool:
    """
    Given these points return if a human or robot could
    actually achieve this position. For example, is this
    arm abnormally long?
    """
    pass


def check_thresholds(new_points: arm_markers) -> bool:
    """
    Using the old_points, check if these new_points
    exceed our allowed speed and directional limits.
    """
    pass


def check_safe_zones(points: arm_markers) -> bool:
    """
    Given the points, check if these points
    encroach our human, robot, and environmental safe zones.
    """
    pass

from typing import List

arm_markers = List[float]
old_points: arm_markers = list()


def preprocessing(new_points: arm_markers) -> arm_markers:
    """
    Using the old_points, check if these new_points
    exceed our allowed speed and directional limits.
    """
    check_feasability()
    smoothing(new_points)
    check_safe_zones(new_points)
    check_thresholds(new_points)

    pass


def check_feasability(points: arm_markers) -> bool:
    """
    Given these points return if a human or robot could
    actually achieve this position. For example, is this
    arm abnormally long?
    """
    pass


def smoothing(new_points: arm_markers) -> arm_markers:
    """
    Using the old_points, smooth these new_points.
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

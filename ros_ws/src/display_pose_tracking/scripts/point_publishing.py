from typing import Dict
from model import RobotAngles


def publish_points(robot_angles: Dict[int, RobotAngles]):
    """
    On a regular interval, publish the next unpublished
    RobotAngles if they are available, or no-op otherwise.
    """
    pass

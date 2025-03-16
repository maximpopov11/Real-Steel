from typing import Any, Dict
from model import Bodypoints

Frame = Any
"""
Video frame.
"""


def publish_frame(frame_queue: Dict[int, Frame]):
    """
    Using a camera, publish video frames to the
    given indexed queue at regular intervals.
    """
    pass


def get_bodypoints(frame: Frame) -> Bodypoints:
    """
    Get raw bodypoints using Mediapipe.
    """
    pass

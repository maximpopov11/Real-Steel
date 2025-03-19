from custom_types import Frame_Pair_t, q_Bodypoints_t

def get_bodypoints(frame_pair: Frame_Pair_t, bodypoints_queue: q_Bodypoints_t):
    """
    Get raw bodypoints using Mediapipe.
    """
    pass

"""
from typing import Dict
import pyrealsense2 as rs
import numpy as np
from time import time
from model import Bodypoints
from util import MP, setup_depth_pipeline, make_detect_async_callback, detect_with_callback, MP_FULL_MODEL_PATH
from custom_types import *

def publish_frame(rospy, frame_queue: Dict[int, Frame_Pair]):
    Using a camera, publish video frames to the
    given indexed queue at regular intervals.

    align_to = rs.stream.color
    align = rs.align(align_to)
    pipeline = setup_depth_pipeline()
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image

        color_frame = frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            continue

        # Convert color frame to numpy array
        color_frame_array = np.asanyarray(color_frame.get_data())

        frame_queue[int(time() * 1000)] = (color_frame_array, depth_frame)


def get_bodypoints(frame_pair : Frame_Pair, bodypoints_queue: Dict[int, Bodypoints]) -> Bodypoints:
    
    Get raw bodypoints using Mediapipe.
    mp_image = MP.Image(image_format=MP.ImageFormat.SRGB, data=frame_pair[0])
    model_path = MP_FULL_MODEL_PATH
    callback = make_detect_async_callback(frame_pair[1], bodypoints_queue)
    
    detect_with_callback(mp_image, model_path, callback)

"""
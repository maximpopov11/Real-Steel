from custom_types import q_Frame_Pair_t
import pyrealsense2 as rs
from util import CAMERA_IMAGE_HEIGHT, CAMERA_IMAGE_WIDTH, timestamp
import rospy
import numpy as np
from heapq import heappush


def setup_depth_pipeline():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("Must have RGB Depth Camera")
        exit(0)

    config.enable_stream(rs.stream.depth, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline


def publish_frame(frame_queue: q_Frame_Pair_t):
    """
    Using a camera, publish video frames to the
    given heap queue at regular intervals.
    """

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
        heappush(frame_queue, (timestamp(), (color_frame_array, depth_frame)))


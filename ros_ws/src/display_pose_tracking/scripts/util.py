import pyrealsense2 as rs
from time import time
import mediapipe as MP
from time import time
from _collections_abc import Callable
import numpy.typing as nptyping

mp_pose = MP.solutions.pose
PoseLandmarkerResult = MP.tasks.vision.PoseLandmarkerResult
BaseOptions = MP.tasks.BaseOptions
PoseLandmarker = MP.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = MP.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = MP.tasks.vision.RunningMode
mpImage = MP.Image

Result_Depth_Callback = Callable[[PoseLandmarkerResult, mpImage, int], None]
"""
Type of the callback that receives the result of mediapipe detect_async.
"""

CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480

LIB_DIR_PATH = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel/ros_ws/src/display_pose_tracking/lib"
MP_LITE_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_lite.task"
MP_FULL_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_full.task"
MP_HEAVY_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_heavy.task"

LIB_DIR_PATH = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel/ros_ws/src/display_pose_tracking/lib"
MP_LITE_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_lite.task"
MP_FULL_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_full.task"
MP_HEAVY_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_heavy.task"

def timestamp():
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

def setup_depth_pipeline():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    #device_product_line = str(device.get_info(rs.camera_info.product_line))

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

def make_detect_async_callback(depth_frame, bodypoints_queue):
    def callback(result: PoseLandmarkerResult, output_image : MP.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        landmarks = result[0]
        print(f"Landmarks: {landmarks}")

        landmarks_with_depth = []
        for point in landmarks:
            x_coord = round(point.x * CAMERA_IMAGE_WIDTH)
            y_coord = round(point.y * CAMERA_IMAGE_WIDTH)
            if x_coord >= CAMERA_IMAGE_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_IMAGE_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                landmarks_with_depth.append((x_coord, y_coord, 0))
                return

            z_coord = depth_frame.get_distance(x_coord, y_coord)
            landmarks_with_depth.append((x_coord, y_coord, z_coord))
            print(f"READ ({int(time()*1000) - timestamp_ms}ms): {landmarks_with_depth}")

        bodypoints_queue[timestamp_ms] = landmarks_with_depth

    return callback

def detect_with_callback(mp_image : nptyping.NDArray, model_path : str, callback : Result_Depth_Callback):
    with PoseLandmarker.create_from_options(PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=callback
    )) as landmarker:
        landmarker.detect_async(mp_image, int(time() * 1000))

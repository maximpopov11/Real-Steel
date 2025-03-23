from time import time

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

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

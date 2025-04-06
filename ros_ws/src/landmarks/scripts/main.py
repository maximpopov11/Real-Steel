import rospy
from custom_msg.msg import Landmarks
from util import *
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import os

from time import time

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
MIN_POSE_DETECTION_CONFIDENCE = .95

curr_dir = os.getcwd()

LIB_DIR_PATH = os.path.join(curr_dir, "src/landmarks/lib")
MP_LITE_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_lite.task"
MP_FULL_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_full.task"
MP_HEAVY_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_heavy.task"

COMBINED_PREPROCESSING_LANDMARS = False

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

def get_relevant_landmarks(result):
    """
    Given a PoseLandmarker result, extract the relevant points we care about.
    Returns a list containing, in order:
    - 0 left hip
    - 1 right hip
    - 2 left shoulder
    - 3 right shoulder
    - 4 left elbow
    - 5 right elbow
    - 6 left wrist
    - 7 right wrist
    - 8 left pinky
    - 9 right pinky
    - 10 left index
    - 11 right index
    - 12 left thumb
    - 13 right thumb
    - 14 nose
    """
    landmarks = result.pose_landmarks[0]
    return [
            landmarks[23], 
            landmarks[24], 
            landmarks[11], 
            landmarks[12], 
            landmarks[13], 
            landmarks[14], 
            landmarks[15], 
            landmarks[16],
            landmarks[17],
            landmarks[18],
            landmarks[19],
            landmarks[20],
            landmarks[21],
            landmarks[22],
            landmarks[0]
        ]



mp_pose = mp.solutions.pose
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
mpImage = mp.Image

align_to = rs.stream.color
align = rs.align(align_to)

def configure_pipeline():
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
        print("Must RGB Depth Camera")
        exit(0)

    config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline

# Setup publishers if we want to debug preprocessing or have combined
def setup():
    if not COMBINED_PREPROCESSING_LANDMARS:
        pub = rospy.Publisher('landmarks', Landmarks, queue_size=10)
    else:
        pub = rospy.Publisher('preprocessed', Landmarks, queue_size=10)

depth_frame_dict = {}

def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
    if len(result.pose_landmarks) == 0:
        rospy.loginfo("Returning, no landmarks found for " + str(timestamp_ms))
        depth_frame_dict.pop(timestamp_ms)
        return

    relevant_landmarks = get_relevant_landmarks(result)

    landmarks_with_depth = []
    depth_frame = depth_frame_dict.pop(timestamp_ms)
    for point in relevant_landmarks:
        x_coord = round(point.x * CAMERA_WIDTH)
        y_coord = round(point.y * CAMERA_HEIGHT)

        if x_coord >= CAMERA_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_HEIGHT:
            rospy.logwarn(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
            landmarks_with_depth.append((x_coord, y_coord, 0))
            continue

        z_coord = depth_frame.get_distance(x_coord, y_coord)
        landmarks_with_depth.append((x_coord, CAMERA_HEIGHT - y_coord, z_coord))
        rospy.loginfo(f"Pose detection took {timestamp() - timestamp_ms}ms!")

    msg = Landmarks()
    msg.left_hip        = landmarks_with_depth[0]
    msg.right_hip       = landmarks_with_depth[1]
    msg.left_shoulder   = landmarks_with_depth[2]
    msg.right_shoulder  = landmarks_with_depth[3]
    msg.left_elbow      = landmarks_with_depth[4]
    msg.right_elbow     = landmarks_with_depth[5]
    msg.left_wrist      = landmarks_with_depth[6]
    msg.right_wrist     = landmarks_with_depth[7]
    msg.left_pinky      = landmarks_with_depth[8]
    msg.right_pinky     = landmarks_with_depth[9]
    msg.left_index      = landmarks_with_depth[10]
    msg.right_index     = landmarks_with_depth[11]
    msg.left_thumb      = landmarks_with_depth[12]
    msg.right_thumb     = landmarks_with_depth[13]
    msg.nose            = landmarks_with_depth[14]
    msg.timestamp       = timestamp_ms

    if not COMBINED_PREPROCESSING_LANDMARS:
        rospy.loginfo(msg) # will continue to log in CLI msg being sent
        pub.publish(msg)
    else:
        process_bodypoints(msg)

def run():
    rospy.init_node('landmarks', anonymous=True)

    rate = rospy.Rate(10)
    landmarker = PoseLandmarker.create_from_options(PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MP_FULL_MODEL_PATH),
            min_pose_detection_confidence=MIN_POSE_DETECTION_CONFIDENCE,
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=callback
    ))

    # Depth Camera Pipeline
    pipeline = configure_pipeline()
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        aligned_depth_frame.keep()

        color_frame = frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)
        frame_time = timestamp()
        depth_frame_dict[frame_time] = aligned_depth_frame
        landmarker.detect_async(mp_image, frame_time)
        rate.sleep()

if __name__ == '__main__':
    try:
        # use flat -combine to run both as same node
        if len(sys.argv) > 1:
            if sys.argv[1] == '-combined':
                COMBINED_PREPROCESSING_LANDMARS = True
                setup_pubs()
        run()
    except rospy.ROSInterruptException:
        pass

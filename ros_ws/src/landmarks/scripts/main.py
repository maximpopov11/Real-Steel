import rospy
from custom_msg.msg import Landmarks
from time import time
from lib.util import MP_LITE_MODEL_PATH, CAMERA_WIDTH, CAMERA_HEIGHT, get_left_arm_landmarks, get_right_arm_landmarks, get_relevant_landmarks
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs

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

pub = rospy.Publisher('landmarks', Landmarks, queue_size=10)
depth_frame_dict = {}

def make_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        relevant_landmarks = get_relevant_landmarks(result)
        print(f"Arm: {relevant_landmarks}")

        landmarks_with_depth = []
        for point in relevant_landmarks:
            x_coord = round(point.x * CAMERA_WIDTH)
            y_coord = round(point.y * CAMERA_HEIGHT)
            if x_coord >= CAMERA_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                landmarks_with_depth.append((x_coord, y_coord, 0))
                continue

            z_coord = depth_frame.get_distance(x_coord, y_coord)
            landmarks_with_depth.append((x_coord, y_coord, z_coord))
            print(f"READ ({int(time()*1000) - timestamp_ms}ms): {landmarks_with_depth}")

        msg = Landmarks()
        msg.left_hip =          landmarks_with_depth[0]
        msg.right_hip =         landmarks_with_depth[1]
        msg.left_shoulder =     landmarks_with_depth[2]
        msg.right_shoulder =    landmarks_with_depth[3]
        msg.left_elbow =        landmarks_with_depth[4]
        msg.right_elbow =       landmarks_with_depth[5]
        msg.left_wrist =        landmarks_with_depth[6]
        msg.right_wrist =       landmarks_with_depth[7]
        msg.left_pinky =        landmarks_with_depth[8]
        msg.right_pinky =       landmarks_with_depth[9]
        msg.left_index =        landmarks_with_depth[10]
        msg.right_index =       landmarks_with_depth[11]
        msg.left_thumb =        landmarks_with_depth[12]
        msg.right_thumb =       landmarks_with_depth[13]
        msg.nose =              landmarks_with_depth[14]

        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        pub.publish(msg)

    return callback

def run():

    rospy.init_node('landmarks', anonymous=True)

    rate = rospy.Rate(10)

    # Depth Camera Pipelin
    pipeline = configure_pipeline()
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image

        color_frame = frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)

        with PoseLandmarker.create_from_options(PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MP_LITE_MODEL_PATH),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=make_callback(aligned_depth_frame)
        )) as landmarker:
            landmarker.detect_async(mp_image, int(time() * 1000))

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

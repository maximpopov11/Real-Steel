import rospy
from custom_msg.msg import arm
#from lib.mp_util import PoseLandmarker, PoseLandmarkerOptions, PoseLandmarkerResult, BaseOptions, VisionRunningMode, get_left_arm_landmarks, mp, dir_path
#from lib.depth_cam import configure_pipeline
from time import time
import numpy as np
import mediapipe as mp

mp = mp
mp_pose = mp.solutions.pose
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
mpImage = mp.Image

dir_path = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel/ros_ws/src/robot_side"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

"""
Given a PoseLandmarker result, extract the data for the left shoulder, elbow, and wrist.
Returns a list containing only those three
"""
def get_left_arm_landmarks(result : PoseLandmarkerResult):
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15]]

"""
Given a PoseLandmarker result, extract the data for the right shoulder, elbow, and wrist.
Returns a list containing only those three
"""
def get_right_arm_landmarks(result : PoseLandmarkerResult):
    landmarks = result.pose_landmarks[0]
    return [landmarks[12], landmarks[14], landmarks[16]]
"""
Given a PoseLandmarker result, extract the data for the arms.
Returns a list containing, in order, left shoulder elbow wrist right shoulder elbow wrist
"""
def get_arm_landmarks(result : PoseLandmarkerResult):
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15], landmarks[12], landmarks[14], landmarks[16]]


import pyrealsense2 as rs

def configure_pipeline():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline

pub = rospy.Publisher('arms', arm, queue_size=10)

def make_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        arm_landmarks = get_arm_landmarks(result)
        print(f"Arm: {arm_landmarks}")

        arms_with_depth = []
        for point in arm_landmarks:
            x_coord = round(point.x * 640)
            y_coord = round(point.y * 480)
            if x_coord >= CAMERA_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                return

            z_coord = depth_frame[y_coord][x_coord]
            arms_with_depth.append((x_coord, y_coord, z_coord))
            print(f"READ ({int(time()*1000) - timestamp_ms}ms): {arms_with_depth}")

        msg = arm()
        msg.left_shoulder = arms_with_depth[0]
        msg.left_elbow = arms_with_depth[1]
        msg.left_wrist = arms_with_depth[2]
        msg.right_shoulder = arms_with_depth[3]
        msg.right_elbow = arms_with_depth[4]
        msg.right_wrist = arms_with_depth[5]

        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        print("sent!")
        pub.publish(msg)


    return callback

def run():
    
    rospy.init_node('robot_camera', anonymous=True)

    rate = rospy.Rate(10)

    model_path = f"{dir_path}/lib/pose_landmarker_lite.task"

    # Depth Camera Pipelin
    pipeline = configure_pipeline()
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)

        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=make_callback(depth_image)
        )

        with PoseLandmarker.create_from_options(options) as landmarker:
            landmarker.detect_async(mp_image, int(time() * 1000))

        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
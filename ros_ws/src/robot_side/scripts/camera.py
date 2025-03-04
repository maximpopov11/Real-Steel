import rospy
from custom_msg.msg import landmarks
#from lib.mp_util import PoseLandmarker, PoseLandmarkerOptions, PoseLandmarkerResult, BaseOptions, VisionRunningMode, get_left_arm_landmarks, mp, dir_path
#from lib.depth_cam import configure_pipeline
from time import time
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs

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

align_to = rs.stream.color
align = rs.align(align_to)


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
Returns a list containing, in order:
- left hip
- right hip
- left shoulder
- right shoulder
- left elbow
- right elbow
- left wrist
- right wrist
- left pinky
- right pinky
- left index
- right index
- left thumb
- right thumb
- nose
"""
def get_relevant_landmarks(result : PoseLandmarkerResult):
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
        print("Must RGB Depth Camera")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline

pub = rospy.Publisher('landmarks', landmarks, queue_size=10)

def make_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        relevant_landmarks = get_relevant_landmarks(result)
        print(f"Arm: {relevant_landmarks}")

        landmarks_with_depth = []
        for point in relevant_landmarks:
            x_coord = round(point.x * 640)
            y_coord = round(point.y * 480)
            if x_coord >= CAMERA_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                landmarks_with_depth.append((x_coord, y_coord, 0))
                return

            z_coord = depth_frame.get_distance(x_coord, y_coord)
            landmarks_with_depth.append((x_coord, y_coord, z_coord))
            print(f"READ ({int(time()*1000) - timestamp_ms}ms): {landmarks_with_depth}")

        msg = landmarks()
        msg.left_hip = landmarks_with_depth[0]
        msg.right_hip = landmarks_with_depth[1]
        msg.left_shoulder = landmarks_with_depth[2]
        msg.right_shoulder = landmarks_with_depth[3]
        msg.left_elbow = landmarks_with_depth[4]
        msg.right_elbow = landmarks_with_depth[5]
        msg.left_wrist = landmarks_with_depth[6]
        msg.right_wrist = landmarks_with_depth[7]
        msg.left_pinky = landmarks_with_depth[8]
        msg.right_pinky = landmarks_with_depth[9]
        msg.left_index = landmarks_with_depth[10]
        msg.right_index = landmarks_with_depth[11]
        msg.left_thumb = landmarks_with_depth[12]
        msg.right_thumb = landmarks_with_depth[13]
        msg.nose = landmarks_with_depth[14]


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
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image

        color_frame = frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)

        with PoseLandmarker.create_from_options(PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=make_callback(aligned_depth_frame)
        )) as landmarker:
            landmarker.detect_async(mp_image, int(time() * 1000))

        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        
        #pub.publish(msg)
        #rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
import rospy
from custom_msg.msg import arm
from lib.mp_util import PoseLandmarker, PoseLandmarkerOptions, PoseLandmarkerResult, BaseOptions, VisionRunningMode, get_left_arm_landmarks, mp, dir_path
from lib.depth_cam import configure_pipeline
from time import time
import numpy as np

pub = rospy.Publisher('left_arm', arm, queue_size=10)

def make_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        left_arm = get_left_arm_landmarks(result)
        print(f"Left Arm: {left_arm}")

        left_shoulder_x = round(left_arm[0].x * 640)
        left_shoulder_y = round(left_arm[0].y * 480)
        left_shoulder_z = 0
        if (left_shoulder_x > 640 or left_shoulder_x < 0) and (left_shoulder_y < 0 or left_shoulder_x > 480):
            left_shoulder_z = depth_frame[left_shoulder_y][left_shoulder_x]
            pub
        else:
            print("OUT OF BOUNDS")

        print(f"read landmarks! ({left_shoulder_x}, {left_shoulder_y}, {left_shoulder_z})")
        msg = arm()
        msg.shoulder = [left_shoulder_x, left_shoulder_y, left_shoulder_z]
        msg.elbow = [0,0,0]
        msg.wrist = [0,0,0]

        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        
        pub.publish(msg)


    return callback

def run():
    
    rospy.init_node('robot_camera', anonymous=True)

    rate = rospy.Rate(10)

    model_path = f"{dir_path}/lib/pose_landmarker_full.task"

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
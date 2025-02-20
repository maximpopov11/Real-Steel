import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from lib.draw import draw_landmarks_on_image
import cv2
from PIL import Image
import numpy as np
import cv2
from time import time
from lib.depth_cam import configure_pipeline
from lib.landmark_util import getLeftArmLandmarks

mp_pose = mp.solutions.pose
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

def make_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        leftArm = getLeftArmLandmarks(result)
        print(leftArm)
        #Xs, Ys, Zs = [], [], []
        #for mark in leftArm:
        #    Xs.append(mark[0])
        #    Ys.append(mark[1])
        #    Zs.append(mark[2])
        leftShoulderX = round(leftArm[0].x * 640)
        leftShoulderY = round(leftArm[0].y * 480)
        leftShoulderZ = 0
        if (leftShoulderX > 640 or leftShoulderX < 0) and (leftShoulderY < 0 or leftShoulderX > 480):
            leftShoulderZ = depth_frame[leftShoulderY][leftShoulderX]
        else:
            print("OUT OF BOUNDS")

        print(f"read landmarks! ({leftShoulderX}, {leftShoulderY}, {leftShoulderZ})")
        #print("I know about depth data too!", depth_frame)

    return callback



def read_video():

    dir_path = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel/lib"
    model_path = f"{dir_path}/pose_landmarker_full.task"



    # Depth Camera Pipelin
    pipeline = configure_pipeline()
    done = False
    while not done:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        #cv2.imshow('frame', frame)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        #depth_colormap_dim = depth_colormap.shape
        #color_colormap_dim = color_image.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)
        #mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=make_callback(depth_image)
        )

        print("hello 0")
        with PoseLandmarker.create_from_options(options) as landmarker:
            landmarker.detect_async(mp_image, int(time() * 1000))

read_video()
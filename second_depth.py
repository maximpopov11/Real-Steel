import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from draw import draw_landmarks_on_image
import cv2
from PIL import Image
import numpy as np
import cv2
import time
from sys import exit
from depth_cam import configure_pipeline

#cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
#cap.set(cv2.CAP_PROP_FRAME_COUNT, 1)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, 10000000)

#if not cap.isOpened():
#    print("Error opening video stream or file")
#    exit()

mp_pose = mp.solutions.pose
#from google.colab.patches import cv2_imshow

dir_path = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel"

model_path = f"{dir_path}/pose_landmarker_full.task"

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

def print_result(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
    annotated = draw_landmarks_on_image(output_image.numpy_view(), result)
        
    img = Image.fromarray(cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR).astype('uint8'))
    img.save(f"./test_{timestamp_ms}.jpeg")  

    print("hello")
    #Image.fromarray(annotated.astype('uint8')).save("annotated.jpeg")
    #cv2.imshow("test", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    #cv2.waitKey(1)    

    print("pose landmarker result: {}".format(result))

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result
)

pipeline = configure_pipeline()
done = False
with PoseLandmarker.create_from_options(options) as landmarker:
    while not done:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        done = True
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        #cv2.imshow('frame', frame)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # landmarker is initialized, can use it here
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)
        #mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
        print("hello 0")
        landmarker.detect_async(mp_image, int(time.time() * 1000))

#read_and_detect()
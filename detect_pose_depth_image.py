import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from lib.draw import draw_landmarks_on_image
import cv2
from PIL import Image
import numpy as np
import time
import cv2
import time
from sys import exit
from lib.depth_cam import configure_pipeline

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
    img.save(f"./2025-02-17_{timestamp_ms}.jpeg")  

    print("hello")
    #Image.fromarray(annotated.astype('uint8')).save("annotated.jpeg")
    #cv2.imshow("test", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    #cv2.waitKey(1)    

    #print("pose landmarker result: {}".format(result))
    with open("./result.txt", 'w') as fp:
        fp.write("pose landmarker result: {}".format(result))
        
    leftShoulderX = round(result.pose_landmarks[0][11].x * 640)
    leftShoulderY = round(result.pose_landmarks[0][11].y * 480)
    #leftShoulderZ = depth_image[leftShoulderY][leftShoulderX]
    print(f"{result.pose_landmarks[0][11]} \n({leftShoulderX}, {leftShoulderY}, {leftShoulderZ})")


    with open("./frame.txt", 'w') as fp:
        fp.write("img: {}\nlen: {}".format(depth_image[100], len(depth_image)))


options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result
)
framesCount = 0

pipeline = configure_pipeline()
depth_image = []
done = False
with PoseLandmarker.create_from_options(options) as landmarker:
    while not done:
        framesCount+=1
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
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

        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow("color", images)
        #cv2.waitKey()    

        #img = Image.fromarray(color_image)
        #img.save(f"./test_02.jpeg")  
        if framesCount > 50:
            # landmarker is initialized, can use it here
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image)
            #mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
    #       print("hello 0")
            landmarker.detect_async(mp_image, int(time.time() * 1000))
            done = True


#read_and_detect()
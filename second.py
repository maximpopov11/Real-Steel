import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from draw import draw_landmarks_on_image
import cv2
from PIL import Image
import cv2
import cv2.legacy
import time
from sys import exit
from secret import DIR_PATH

mp_pose = mp.solutions.pose
# from google.colab.patches import cv2_imshow

model_path = f"{DIR_PATH}/pose_landmarker_lite.task"

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode


def print_result(
    result: PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int
):
    annotated = draw_landmarks_on_image(output_image.numpy_view(), result)

    img = Image.fromarray(annotated.astype("uint8"))
    img.save(f"./test/test2_{timestamp_ms}.jpeg")

    print("hello")
    # Image.fromarray(annotated.astype('uint8')).save("annotated.jpeg")
    # cv2.imshow("test", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    # cv2.waitKey(1)

    print("pose landmarker result: {}".format(result))


options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result,
)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
cap.set(cv2.CAP_PROP_FRAME_COUNT, 1)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 10000000)

if not cap.isOpened():
    print("Error opening video stream or file")
    exit()

with PoseLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Cant receive frame, exiting...")

        # cv2.imshow('frame', frame)
        # landmarker is initialized, can use it here
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        # mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
        print("hello 0")
        landmarker.detect_async(mp_image, int(time.time() * 1000))

read_and_detect()

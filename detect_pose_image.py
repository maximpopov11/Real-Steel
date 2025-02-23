import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from lib.draw import draw_landmarks_on_image
import cv2
from PIL import Image
from secret import DIR_PATH

# from google.colab.patches import cv2_imshow

model_path = f"{DIR_PATH}/pose_landmarker_heavy.task"

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE,
)

with PoseLandmarker.create_from_options(options) as landmarker:
    # landmarker is initialized, can use it here
    mp_image = mp.Image.create_from_file(f"{DIR_PATH}/tkd.jpeg")
    # mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
    result = landmarker.detect(mp_image)
    annotated = draw_landmarks_on_image(mp_image.numpy_view(), result)

    img = Image.fromarray(annotated.astype("uint8"))
    img.save("test_output.jpeg")

    cv2.imshow("test", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    cv2.waitKey()

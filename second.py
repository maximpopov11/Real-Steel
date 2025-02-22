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
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mp_pose = mp.solutions.pose
# from google.colab.patches import cv2_imshow

model_path = f"{DIR_PATH}/pose_landmarker_lite.task"

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode


landmarks = []


def print_result(
    result: PoseLandmarkerResult,
    output_image: mp.Image,
    timestamp_ms: int,
    show_background: bool = False,
):
    # Extract the landmarks from the result
    global landmarks
    landmarks.append(result.pose_landmarks)

    # Convert the image to a numpy array
    output_image_np = output_image.numpy_view()

    if not show_background:
        # Create a blank (black) image with the same size as the original image
        height, width, _ = output_image_np.shape
        output_image_np = np.zeros((height, width, 3), dtype=np.uint8)

    # Draw landmarks on the image (either on the original or blank background)
    annotated = draw_landmarks_on_image(output_image_np, result)

    # Save the annotated image
    # img = Image.fromarray(annotated.astype("uint8"))
    # img.save(f"./test/test2_{timestamp_ms}.jpeg")

    # Optionally display the image
    # cv2.imshow("Annotated Image", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    # cv2.waitKey(1)

    # print("pose landmarker result: {}".format(result))


def plot_landmarks_in_3d(landmark_frames):
    plt.ion()  # Turn on interactive mode
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    skipped = 0

    for landmark_frame in landmark_frames:
        if not landmark_frame:
            # Mediapipe probably didn't find a person in this frame, we can skip it
            skipped += 1
            continue

        landmarks = landmark_frame[0]
        # print(landmarks)

        ax.clear()  # Clear previous points

        # Extract the 3D coordinates (x, y, z)
        x_coords = [landmark.x for landmark in landmarks]  # Left/Right
        y_coords = [landmark.y for landmark in landmarks]  # Up/Down
        z_coords = [landmark.z for landmark in landmarks]  # Forward/Backward

        ax.scatter(x_coords, y_coords, z_coords)

        # Label the axes
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")

        # Adjust the view so we mostly see the XY plane
        ax.view_init(elev=90, azim=90)  # Top-down view looking at the XY plane

        plt.draw()
        plt.pause(0.1)  # Small pause to allow the figure to update

    print(f"Found {len(landmark_frames) - skipped} / {len(landmark_frames)}")


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

count = 0
with PoseLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Cant receive frame, exiting...")

        count += 1
        if count == 100:
            break

        # cv2.imshow('frame', frame)
        # landmarker is initialized, can use it here
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        # mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=numpy_image)
        landmarker.detect_async(mp_image, int(time.time() * 1000))

plot_landmarks_in_3d(landmarks)

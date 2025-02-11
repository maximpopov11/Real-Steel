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
import matplotlib.pyplot as plt

mp_pose = mp.solutions.pose

pose = mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.3, model_complexity=2)

mp_drawing = mp.solutions.drawing_utils

sample = cv2.imread('tkd.jpeg')
plt.figure(figsize=[10, 10])
plt.title("sample")
plt.axis('off')
plt.imshow(sample[:,:,::-1])
plt.show()

results = pose.process(cv2.cvtColor(sample, cv2.COLOR_BGR2RGB))

image_height, image_width, _ = sample.shape
img_copy = sample.copy()

if results.pose_landmarks:
    mp_drawing.draw_landmarks(image=img_copy, landmark_list=results.pose_landmarks, connections=mp_pose.POSE_CONNECTIONS)
    fig = plt.figure(figsize=[10, 10])
    plt.title("sample")
    plt.axis('off')
    plt.imshow(img_copy[:,:,::-1])
    plt.show()
    mp_drawing.plot_landmarks(results.pose_world_landmarks, mp_pose.POSE_CONNECTIONS)
    for i in range(2):
        print(f'{mp_pose.PoseLandmark(i).name}:\n{results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value]}') 
        print(f'x: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].x * image_width}')
        print(f'y: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].y * image_height}')
        print(f'z: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].z * image_width}')
        print(f'visibility: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].visibility}\n')


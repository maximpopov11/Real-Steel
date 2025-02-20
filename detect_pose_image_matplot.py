import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from lib.draw import draw_landmarks_on_image
import cv2
from PIL import Image
import cv2
import time
from sys import exit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

'''
sample = cv2.imread('./images/tkd.jpeg')
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
    for i in range(11, 13):
        print(f'{mp_pose.PoseLandmark(i).name}:\n{results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value]}') 
        print(f'x: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].x * image_width}')
        print(f'y: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].y * image_height}')
        print(f'z: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].z * image_width}')
        print(f'visibility: {results.pose_landmarks.landmark[mp_pose.PoseLandmark(i).value].visibility}\n')
'''

def detect_pose(image, pose, display=True):
    output_image = image.copy()
    imageRGB = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

    results = pose.process(imageRGB)

    height, width, _ = output_image.shape

    #landmarks = ([], [], [])
    landmarks = []

    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image=output_image, landmark_list=results.pose_landmarks, connections=mp_pose.POSE_CONNECTIONS)

        for landmark in results.pose_landmarks.landmark:
            #landmarks[0].append(int(landmark.x*width))
            #landmarks[1].append(int(landmark.y*height))
            #landmarks[2].append(int(landmark.z*width))
            landmarks.append((int(landmark.x*width), int(landmark.y*height), int(landmark.z*width)))

    if display:
        plt.figure(figsize=[22,22])
        plt.subplot(121); plt.imshow(image[:,:,::-1]); plt.title("Original Image"); plt.axis('off')
        plt.subplot(122); plt.imshow(output_image[:,:,::-1]); plt.title("Output Image"); plt.axis('off')

        mp_drawing.plot_landmarks(results.pose_world_landmarks, mp_pose.POSE_CONNECTIONS)
        
    else:
        return output_image, landmarks
    

pose = mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.3, model_complexity=2)
detect_pose(cv2.imread('./images/tkd.jpeg'), pose)

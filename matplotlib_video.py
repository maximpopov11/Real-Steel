import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from lib.draw import draw_landmarks_on_image
import cv2
from PIL import Image
import cv2
from time import time
from sys import exit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from detect_pose_image_matplot import detectPose
from matplotlib.animation import FuncAnimation
from lib.landmark_util import getLeftArmLandmarks

mp_pose = mp.solutions.pose

pose_video = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.3, model_complexity=2)
def videoDetection():
    video = cv2.VideoCapture(0)
    time1 = 0

    while video.isOpened():
        ok, frame = video.read()
        if not ok:
            break
        frame_height, frame_width, _ = frame.shape
        frame, landmarks = detectPose(frame, pose_video, display=False)

        time2 = time()
        if (time2 - time1) > 0:
            fps = 1.0 / (time2 - time1)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 3)
            cv2.imshow('Pose detecting!', frame)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
    video.release()
    cv2.destroyAllWindows()

#videoDetection()
#fig = plt.figure()

#ax = fig.add_subplot(111, projection="3d")
def getFrame(frame):
    video = cv2.VideoCapture(0)
#    time1 = 0

    ok, frame = video.read()
    if not ok:
        print("error!")
        return
    frame_height, frame_width, _ = frame.shape
    frame, landmarks = detectPose(frame, pose_video, display=False)
    leftArm = getLeftArmLandmarks(landmarks)
    Xs, Ys, Zs = [], [], []
    for mark in leftArm:
        Xs.append(mark[0])
        Ys.append(mark[1])
        Zs.append(mark[2])

    print("read landmarks!", leftArm)
    ax.clear()
#    ax.scatter(Xs, Ys, Zs)
    ax.plot(xs=Xs, ys=Ys, zs=Zs)
    return landmarks


#ani = FuncAnimation(fig, getFrame, interval=100)

#plt.show()
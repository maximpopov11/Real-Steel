import mediapipe as mp

mp = mp
mp_pose = mp.solutions.pose
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
mpImage = mp.Image

dir_path = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel"


"""
Given a PoseLandmarker result, extract the data for the left shoulder, elbow, and wrist.
Returns a list containing only those three
"""
def get_left_arm_landmarks(result : PoseLandmarkerResult):
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15]]


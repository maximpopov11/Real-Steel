import mediapipe.tasks.vision.PoseLandmarkerResult as PoseLandmarkerResult
"""
Given a PoseLandmarker result, extract the data for the left shoulder, elbow, and wrist.
Returns a list containing only those three
"""
def get_left_arm_landmarks(result : PoseLandmarkerResult):
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15]]

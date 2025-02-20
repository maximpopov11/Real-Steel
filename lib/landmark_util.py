
def getLeftArmLandmarks(result):
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15]]

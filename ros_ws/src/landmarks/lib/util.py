CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

LIB_DIR_PATH = "/home/zhc/Documents/ISU/cs402/sd15_reel-steel/ros_ws/src/landmarks/lib"
MP_LITE_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_lite.task"
MP_FULL_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_full.task"
MP_HEAVY_MODEL_PATH = f"{LIB_DIR_PATH}/pose_landmarker_heavy.task"

def get_left_arm_landmarks(result):
    """
    Given a PoseLandmarker result, extract the data for the left shoulder, elbow, and wrist.
    Returns a list containing only those three
    """
    landmarks = result.pose_landmarks[0]
    return [landmarks[11], landmarks[13], landmarks[15]]


def get_right_arm_landmarks(result):
    """
    Given a PoseLandmarker result, extract the data for the right shoulder, elbow, and wrist.
    Returns a list containing only those three
    """
    landmarks = result.pose_landmarks[0]
    return [landmarks[12], landmarks[14], landmarks[16]]

def get_relevant_landmarks(result):
    """
    Given a PoseLandmarker result, extract the relevant points we care about.
    Returns a list containing, in order:
    - 0 left hip
    - 1 right hip
    - 2 left shoulder
    - 3 right shoulder
    - 4 left elbow
    - 5 right elbow
    - 6 left wrist
    - 7 right wrist
    - 8 left pinky
    - 9 right pinky
    - 10 left index
    - 11 right index
    - 12 left thumb
    - 13 right thumb
    - 14 nose
    """
    landmarks = result.pose_landmarks[0]
    return [
            landmarks[23], 
            landmarks[24], 
            landmarks[11], 
            landmarks[12], 
            landmarks[13], 
            landmarks[14], 
            landmarks[15], 
            landmarks[16],
            landmarks[17],
            landmarks[18],
            landmarks[19],
            landmarks[20],
            landmarks[21],
            landmarks[22],
            landmarks[0]
        ]


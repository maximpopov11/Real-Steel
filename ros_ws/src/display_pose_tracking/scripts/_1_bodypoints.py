from custom_types import ts_Frame_Pair_t, q_Bodypoints_t
from util import MP_FULL_MODEL_PATH, CAMERA_IMAGE_HEIGHT, CAMERA_IMAGE_WIDTH
import numpy.typing as nptyping
from heapq import heappush
import mediapipe as MP
from _collections_abc import Callable


mp_pose = MP.solutions.pose
PoseLandmarkerResult = MP.tasks.vision.PoseLandmarkerResult
BaseOptions = MP.tasks.BaseOptions
PoseLandmarker = MP.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = MP.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = MP.tasks.vision.RunningMode
mpImage = MP.Image

Result_Depth_Callback = Callable[[PoseLandmarkerResult, mpImage, int], None]
"""
Type of the callback that receives the result of mediapipe detect_async.
"""


def make_detect_async_callback(depth_frame, bodypoints_queue):
    """
    Given a depth frame and a bodypoints_queue, form a closure and create a function 
    that will use said depth frame and queue to add depth coordinates to the PoseLandmarkerResult, and then add the resulting
    Bodypoints_t to the queue.
    """

    def callback(result: PoseLandmarkerResult, output_image : MP.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        landmarks = result[0]
        print(f"Landmarks: {landmarks}")

        landmarks_with_depth = []
        for point in landmarks:
            x_coord = round(point.x * CAMERA_IMAGE_WIDTH)
            y_coord = round(point.y * CAMERA_IMAGE_WIDTH)
            if x_coord >= CAMERA_IMAGE_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_IMAGE_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                landmarks_with_depth.append((x_coord, y_coord, 0))
                return

            z_coord = depth_frame.get_distance(x_coord, y_coord)
            landmarks_with_depth.append((x_coord, y_coord, z_coord))
            #print(f"READ ({timestamp_ms() - timestamp_ms}ms): {landmarks_with_depth}")
        heappush(bodypoints_queue, (timestamp_ms, landmarks_with_depth))

    return callback

def detect_with_callback(timestamp : int, mp_image : nptyping.NDArray, model_path : str, callback : Result_Depth_Callback):
    """
    Creates a Mediapipe PoseLandmarker with the provided model and callback function, and calls detect_async on it
    """
    with PoseLandmarker.create_from_options(PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=callback
    )) as landmarker:
        landmarker.detect_async(mp_image, timestamp)


def get_bodypoints(frame_pair: ts_Frame_Pair_t, bodypoints_queue: q_Bodypoints_t):
    """
    Given a timestamp and the corresponding Frame_Pair, asynchronously generate Bodypoints and load them into the provided bodypoints_queue.
    """
    mp_image = MP.Image(image_format=MP.ImageFormat.SRGB, data=frame_pair[1])
    callback = make_detect_async_callback(frame_pair[1], bodypoints_queue)
    
    detect_with_callback(frame_pair[0], mp_image, MP_FULL_MODEL_PATH, callback)

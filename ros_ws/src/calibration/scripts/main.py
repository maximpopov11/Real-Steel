from custom_msg.msg import Landmarks
from typing import List
import bisect
import math
import rospy
import statistics


# A sorted list of calculated arm lengths from frames
_left_arm_lengths: List[float] = []
_right_arm_lengths: List[float] = []


def get_left_arm_length() -> float:
    """
    Get the maximum found left arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    return _get_arm_length(_left_arm_lengths)


def get_right_arm_length() -> float:
    """
    Get the maximum found right arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    return _get_arm_length(_right_arm_lengths)


def _get_arm_length(sorted_lengths: List[float]) -> float:
    """
    Get the maximum found arm length excluding outliers expected to be caused by camera or mediapipe innacuracy.
    """

    if not sorted_lengths:
        return 0.0

    if len(sorted_lengths) < 4:
        # If not enough data to meaningfully detect outliers, return max
        return sorted_lengths[-1]

    # Sort the lengths to compute quartiles
    q1 = statistics.quantiles(sorted_lengths, n=4)[0]  # 25th percentile
    q3 = statistics.quantiles(sorted_lengths, n=4)[2]  # 75th percentile
    iqr = q3 - q1

    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr

    # Filter out outliers
    filtered = [l for l in sorted_lengths if lower_bound <= l <= upper_bound]

    if not filtered:
        return sorted_lengths[-1]  # fallback if everything was removed

    return sorted_lengths[-1]


def _calibrate(msg):
    """
    Subscribe to a ROS topic to receive bodypoints for calibration.

    Args:
        msg: ROS message in the form of msg.Landmarks

    Returns:
        None. Populates _left_arm_lengths and _right_arm_lengths.
    """

    left_points = [msg.left_shoulder, msg.left_elbow, msg.left_wrist]
    right_points = [msg.right_shoulder, msg.right_elbow, msg.right_wrist]

    left_length = _get_length(left_points)
    right_length = _get_length(right_points)

    bisect.insort(_left_arm_lengths, left_length)
    bisect.insort(_right_arm_lengths, right_length)


def _get_length(points: List[List[float]]) -> float:
    """
    Get the length of the ordered set of 3D points.
    """

    if len(points) < 2:
        return 0.0

    total_length = 0.0
    for i in range(1, len(points)):
        p1, p2 = points[i - 1], points[i]
        distance = math.sqrt(
            (p2[0] - p1[0]) ** 2 +
            (p2[1] - p1[1]) ** 2 +
            (p2[2] - p1[2]) ** 2
        )
        total_length += distance

    return total_length


def _app():
    rospy.Subscriber('calibration', Landmarks, _calibrate)
    rospy.init_node('preprocessing', anonymous=True)
    #pubtest()
    rospy.spin()


if __name__ == '__main__':
    try:
        _app()
    except rospy.ROSInterruptException:
        pass

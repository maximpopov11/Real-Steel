from time import time

SPEED_THRESHOLD = 1.0  # rad/s

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

def interpolate_points(start_angles, end_angles, num_steps):
    if num_steps <= 0:
        return []
    interpolated = []
    for step in range(1, num_steps + 1):
        alpha = step / num_steps  # Ranges from 1/num_steps to 1.0
        # Linear interpolation for each joint: 
        # angle = start + (end - start) * progress_ratio
        interp = [s + alpha * (e - s) for s, e in zip(start_angles, end_angles)]
        interpolated.append(interp)
    return interpolated

left_arm_joint_names = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint"
]

right_arm_joint_names = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint"
]
from time import time
import numpy as np
import math
import os

import rospy
from custom_msg.msg import Angles

import mujoco
import mujoco.viewer

curr_dir = os.getcwd()

model_path = '../models/unitree_g1/scene.xml'
absolute_path = os.path.join(curr_dir, model_path)

model = mujoco.MjModel.from_xml_path(absolute_path)
data = mujoco.MjData(model)

""" Joints we are currently using (just left/right shoulder & elbow) """
left_arm_joint_names = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint"
]

right_arm_joint_names = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint"
]
""""""

""" Storage for joint id's """
left_actuator_ids = []
right_actuator_ids = []
""""""

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

""" For interpolation of points in simulator """
SPEED_THRESHOLD = 0.5  # radians/s

def interpolate_points_sim(start_angles, end_angles, num_steps):
    if num_steps <= 0:
        return []
    interpolated = []
    for step in range(1, num_steps + 1):
        alpha = step / num_steps    # range from 1/num_steps to 1.0

        # linear interpolation for each joint
        # angle = start + (end - start) * progress_ratio
        interp = [s + alpha * (e-s) for s, e in zip(start_angles, end_angles)]
        interpolated.append(interp)
    return interpolated


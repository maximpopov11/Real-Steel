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
""""""

""" Storage for joint id's """
left_actuator_ids = []
right_actuator_ids = []
""""""

def setup_ids():
    """
    Will find/connect joint id's based on used joint names from left or right arms
    """
    # left arm id's
    for i in range(len(left_arm_joint_names)):
        id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_arm_joint_names[i])
        if (id != -1):
            left_actuator_ids.append(id)
        else:
            rospy.logerr_once(f"Actuator {left_arm_joint_names[i]} not found in model")
            return -1
    # right arm id's
    for i in range(len(right_arm_joint_names)):
        id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, right_arm_joint_names[i])
        if (id != -1):
            right_actuator_ids.append(id)
        else:
            rospy.logerr_once(f"Actuator {right_arm_joint_names[i]} not found in model")
            return -1

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

""" For interpolation of points in simulator """
SPEED_THRESHOLD = 1.0  # radians/s
REACH_THRESHOLD = 0.05  # radians

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


import rospy
from custom_msg.msg import Angles
from sim_util import timestamp

import numpy as np

import mujoco
import mujoco.viewer

# path to where git repo is
home_path = '/home/db-triple/Desktop/git/sd15_reel-steel/' 
model_path = 'models/unitree_g1/scene.xml'
model = mujoco.MjModel.from_xml_path(home_path + model_path)
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
    
def use_angles(msg):
    """
    Placeholder for the robot long-term. Use simulator or other systems to visualize the angle output.
    """
    # print(timestamp(), msg)

    for i in range(len(left_actuator_ids)):
        target_angle = msg.left_arm[i]
        data.ctrl[left_actuator_ids[i]] = target_angle

    for i in range(len(right_actuator_ids)):
        target_angle = msg.right_arm[i]
        data.ctrl[right_actuator_ids[i]] = target_angle

    
def app():
    sub = rospy.Subscriber('robot_angles', Angles, use_angles)
    rospy.init_node('simulator', anonymous=True)

    # if id's of joints are not found exit simulation
    if setup_ids() == -1:
        rospy.logerr_once("Exiting simulation: FAILED to setup joint ID's")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # refresh rate for sim
        rate = rospy.Rate(60) # 60 Hz refresh

        viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.75])
        viewer.cam.distance = 2.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 180

        while not rospy.is_shutdown() and viewer.is_running():
            # step in sim
            mujoco.mj_step(model, data)

            # lock and update viewer
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # sync viewer to display updated sim
            viewer.sync()

            # sleep maintaining rate
            rate.sleep()

        rospy.signal_shutdown("Simulation Exiting")

if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
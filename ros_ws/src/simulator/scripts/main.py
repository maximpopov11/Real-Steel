import rospy
from custom_msg.msg import Angles
from sim_util import timestamp

import os
import sys

import mujoco
import mujoco.viewer

# path to where git repo is
home_path = '/home/db-triple/Desktop/git/sd15_reel-steel/' 
model_path = 'models/unitree_g1/scene.xml'
model = mujoco.MjModel.from_xml_path(home_path + model_path)
data = mujoco.MjData(model)



def use_angles(msg):
    """
    Placeholder for the robot long-term. Use simulator or other systems to visualize the angle output.
    """
    # print(timestamp(), msg)
    print("received msg")

    joint_name = "left_shoulder_pitch_joint"
    target_angle = msg.left_arm[0]

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)

    if (actuator_id == -1):
        print(f"Actuator {joint_name} not found in model")
        return
    
    # set control input to actuator
    data.ctrl[actuator_id] = target_angle

    
def app():
    sub = rospy.Subscriber('robot_angles', Angles, use_angles)
    rospy.init_node('simulator', anonymous=True)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # refresh rate for sim
        rate = rospy.Rate(60) # 60 Hz refresh

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
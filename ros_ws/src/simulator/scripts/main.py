from sim_util import *

last_angles = []

left_interpolated_angles = []
right_interpolated_anlges = []

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

def callback(msg):
    curr_angles = msg.left_arm + msg.right_arm

    if len(last_angles) <= 0:
        last_angles = curr_angles
        left_interpolated_angles.append(msg.left_arm)
        right_interpolated_anlges.append(msg.right_arm)
        return

    time_diff = 0.1 # fixed ratio between callbacks
    speed_per_joint = [
        abs(curr - last) / time_diff
        for curr, last in zip(curr_angles, last_angles)
    ]

    max_speed = max(speed_per_joint)

    if max_speed > SPEED_THRESHOLD:
        required_steps = math.ceil(max_speed / SPEED_THRESHOLD)
        interpolated = interpolate_points_sim(last_angles, curr_angles, required_steps)
        for angles in interpolated:
            left = angles[:5]
            right = angles[5:]

            left_interpolated_angles.append(left)
            right_interpolated_anlges.append(right)

        last_angles = curr_angles
    else:
        left_interpolated_angles.append(msg.left_arm)
        right_interpolated_anlges.append(msg.right_arm)

def use_angles():
    """
    Placeholder for the robot long-term. Use simulator or other systems to visualize the angle output.
    """
    # print(timestamp(), msg)

    if len(left_interpolated_angles) <= 0 or len(right_interpolated_anlges) <= 0:
        return

    output_left = left_interpolated_angles.pop()
    output_right = right_interpolated_anlges.pop()

    for i in range(len(left_actuator_ids)):
        target_angle = output_left[i]
        data.ctrl[left_actuator_ids[i]] = target_angle

    for i in range(len(right_actuator_ids)):
        target_angle = output_right[i]
        data.ctrl[right_actuator_ids[i]] = target_angle


def app():
    sub = rospy.Subscriber('robot_angles', Angles, callback)
    rospy.init_node('simulator', anonymous=False)

    # if id's of joints are not found exit simulation
    if setup_ids() == -1:
        rospy.logerr_once("Exiting simulation: FAILED to setup joint ID's")
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # refresh rate for sim
        rate = rospy.Rate(10) # 10 Hz refresh

        viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.75])   # camera looking at
        viewer.cam.distance = 2.0                           # how far is camera from lookat
        viewer.cam.elevation = -20                          # vertical rotation degrees
        viewer.cam.azimuth = 180                            # horizontal rotation degrees

        while not rospy.is_shutdown() and viewer.is_running():
            # step in sim
            mujoco.mj_step(model, data)

            # lock and update viewer
            # with viewer.lock():
                # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # sync viewer to display updated sim
            viewer.sync()

            rate.sleep()

        rospy.signal_shutdown("Simulation Exiting")

if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

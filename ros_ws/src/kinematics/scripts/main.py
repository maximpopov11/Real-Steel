import rospy
from custom_msg.msg import Landmarks, Angles
from kin_util import timestamp

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_end_effector_link("right_wrist_roll_joint")

csv_file = open("ik_joint_positions.csv", "w", newline="")
writer = csv.writer(csv_file)
joint_names = group.get_joints()
writer.writerow(["Pose Index"] + joint_names)  # Write CSV header

last_joint_values = group.get_current_joint_values()

def generate_angles(msg):
    global last_joint_values  # We need to keep track of the last known joint values
    
    print(timestamp(), msg)

    right_shoulder = msg.right_shoulder
    right_elbow = msg.right_elbow
    right_wrist = msg.right_wrist
    right_hip = msg.right_hip

    relative_wrist_position = [
        right_wrist[0] - right_hip[0],
        right_wrist[1] - right_hip[1],
        right_wrist[2] - right_hip[2]
    ]

    target_pose = Pose()
    target_pose.position.x = relative_wrist_position[0]
    target_pose.position.y = relative_wrist_position[1]
    target_pose.position.z = relative_wrist_position[2]
    target_pose.orientation.w = 1.0  

    group.set_pose_target(target_pose)
    success, plan, _, _ = group.plan()

    if success and plan.joint_trajectory.points:
        last_joint_values = plan.joint_trajectory.points[-1].positions

        angles_msg = Angles()
        angles_msg.right_arm = list(last_joint_values)
        pub.publish(angles_msg)

        writer.writerow(["Next Pose"] + list(last_joint_values))
        print(f"{last_joint_values}")

        robot_state = RobotState()
        robot_state.joint_state.name = group.get_joints()
        robot_state.joint_state.position = last_joint_values
        group.set_start_state(robot_state)  # Force move group to next state
    else:
        print("Failed to compute IK solution")

def app():
    sub = rospy.Subscriber('landmarks', Landmarks, generate_angles)
    rospy.init_node('kinematics', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pdef generate_angles(msg):

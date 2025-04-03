import sys
import rospy
from custom_msg.msg import Landmarks, Angles
from kin_util import timestamp
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

# move group
group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_end_effector_link("right_rubber_hand")
# move_group.set_pose_reference_frame("world")
# move_group.set_num_planning_attempts(1)
# move_group.set_planning_time(.05)  # 20Hz right now
# move_group.allow_replanning(False)
move_group.set_goal_joint_tolerance(0.001)
move_group.set_goal_position_tolerance(0.01)
move_group.set_goal_orientation_tolerance(1)  # don't care about the orientation.?

# CSV file
#csv_file = open("ik_joint_positions.csv", "w", newline="")
#writer = csv.writer(csv_file)
#joint_names = move_group.get_joints()
#writer.writerow(["Pose Index"] + joint_names)  # Write CSV header

#last_joint_values = move_group.get_current_joint_values()


ROBOT_HIP_WIDTH_METERS = 0.25
ROBOT_SHOULDER_HIP_METERS = .27

def compute_distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**(1/2)

def generate_angles(msg):
    # Get the timestamp so we can time how long this takes to run
    before_ts = timestamp()

    # computation is done through a ROS service
    rospy.wait_for_service("compute_ik")
    
    # Assume the points are preprocessed and relative to the midpoint of the hips
    right_vertical_distance = compute_distance(msg.right_shoulder, msg.right_hip)
    left_vertical_distance = compute_distance(msg.left_shoulder, msg.left_hip)
    hips_distance = compute_distance(msg.right_hip, msg.left_hip)

    horizontal_scale = ROBOT_HIP_WIDTH_METERS / hips_distance
    right_vertical_scale = ROBOT_SHOULDER_HIP_METERS / right_vertical_distance
    left_vertical_scale = ROBOT_SHOULDER_HIP_METERS / left_vertical_distance
    z_scale_factor = ROBOT_HIP_WIDTH_METERS

    # Get a position position request and set it up for the right arm and hand
    # This should be updated to make one request for both left and right arms simultaneously.
    right_request = GetPositionIKRequest()
    right_request.ik_request.group_name = "right_arm"
    right_request.ik_request.ik_link_name = "right_rubber_hand"
    
    # Set the pose position xyz:
    # - Round because floating point can be weird (might be able to omit)
    # - Reorder which coordinates are used where, because the robot model is oriented facing in the y direction
    # - Scale the points by either the hip scale factor (for x and y) or the z_scale_factor
    right_request.ik_request.pose_stamped.pose.position.x = round(msg.right_wrist[2]*z_scale_factor, 3)
    right_request.ik_request.pose_stamped.pose.position.y = round(msg.right_wrist[0]*horizontal_scale, 3)
    right_request.ik_request.pose_stamped.pose.position.z = round(msg.right_wrist[1]*right_vertical_scale, 3)
    right_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.w = 1.0

    # Doesn't seem to be working, need to look into
    right_request.ik_request.avoid_collisions = True

    # Repeat all for left arm
    left_request = GetPositionIKRequest()
    left_request.ik_request.group_name = "left_arm"
    left_request.ik_request.ik_link_name = "left_rubber_hand"
    left_request.ik_request.pose_stamped.pose.position.x = round(msg.left_wrist[2]*z_scale_factor, 3)
    left_request.ik_request.pose_stamped.pose.position.y = round(msg.left_wrist[0]*horizontal_scale, 3)
    left_request.ik_request.pose_stamped.pose.position.z = round(msg.left_wrist[1]*left_vertical_scale, 3)
    left_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.w = 1.0
    left_request.ik_request.avoid_collisions = True

    # Debug statements
    print("Hip scale factor", hips_distance, horizontal_scale)
    rospy.loginfo(f"Right Vertical distance + Scale Factor: {right_vertical_distance} {right_vertical_scale}")
    #rospy.loginfo("Scaled wrist", msg.right_wrist[0]*hip_scale_factor, msg.right_wrist[1]*hip_scale_factor, msg.right_wrist[2]*hip_scale_factor)

    # Establish varialbes for the responses
    right_response : GetPositionIKResponse = False
    left_response : GetPositionIKResponse = False
    try:
        right_response = compute_ik(right_request)
        left_response = compute_ik(left_request)
    except ValueError: 
        print("ValueError!")
    
    # If the error code is 1 we successfully did IK
    if right_response.error_code.val == 1 and left_response.error_code.val == 1:
        angles_msg = Angles()
        # the returned position field is a 3-tuple; we need it to be a list. Also cropping out just the 4 angles we  want
        angles_msg.right_arm = [x for x in right_response.solution.joint_state.position[-7:-3]] + [0]
        angles_msg.left_arm = [x for x in left_response.solution.joint_state.position[-14:-10]] + [0]
        pub.publish(angles_msg)
        after_ts = timestamp()
        rospy.loginfo(f"({after_ts - before_ts}ms): Left: {angles_msg.right_arm} Right: {angles_msg.right_arm}")
    else:
        lt = left_request.ik_request.pose_stamped.pose.position
        rt = right_request.ik_request.pose_stamped.pose.position
        lw = msg.left_wrist
        ls = msg.left_shoulder
        rw = msg.right_wrist
        rs = msg.right_shoulder
        rospy.logerr(f"rvs: {right_vertical_scale} | {left_response.error_code.val}, ({lw[0]}, {lw[1]}, {lw[2]}) | {right_response.error_code.val}, ({rw[0]}, {rw[1]}, {rw[2]})")
        rospy.logerr(f"{left_response.error_code.val}, ({lt.x}, {lt.y}, {lt.z}) | {right_response.error_code.val}, ({rt.x}, {rt.y}, {rt.z})")


def app():
    sub = rospy.Subscriber('preprocessed', Landmarks, generate_angles)
    rospy.init_node('kinematics', anonymous=False)

    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass


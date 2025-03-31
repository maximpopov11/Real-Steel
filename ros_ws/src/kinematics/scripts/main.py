import csv
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


ROBOT_HIP_WIDTH_METERS = 0.15

def generate_angles(msg):
    #global last_joint_values  # We need to keep track of the last known joint values
    rospy.wait_for_service("compute_ik")
    print(timestamp(), msg)
    
    # Assume the points are preprocessed and relative to the midpoint of the hips
    right_shoulder = msg.right_shoulder
    right_elbow = msg.right_elbow
    
    right_hip = msg.right_hip
    left_hip = msg.left_hip
    hips_distance = ((right_hip[0] - left_hip[0])**2 + (right_hip[1] - left_hip[1])**2 + (right_hip[2] - left_hip[2])**2)**(1/2)
    hip_scale_factor = ROBOT_HIP_WIDTH_METERS / hips_distance

    right_request = GetPositionIKRequest()
    right_request.ik_request.group_name = "right_arm"
    right_request.ik_request.ik_link_name = "right_rubber_hand"
    right_request.ik_request.pose_stamped.pose.position.x = round(msg.right_wrist[2]*hip_scale_factor, 3)
    right_request.ik_request.pose_stamped.pose.position.y = round(msg.right_wrist[0]*hip_scale_factor, 3)
    right_request.ik_request.pose_stamped.pose.position.z = round(-1*msg.right_wrist[1]*hip_scale_factor, 3)
    right_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.w = 1.0
    right_request.ik_request.avoid_collisions = True

    left_request = GetPositionIKRequest()
    left_request.ik_request.group_name = "left_arm"
    left_request.ik_request.ik_link_name = "left_rubber_hand"
    left_request.ik_request.pose_stamped.pose.position.x = round(msg.left_wrist[2]*hip_scale_factor, 3)
    left_request.ik_request.pose_stamped.pose.position.y = round(msg.left_wrist[0]*hip_scale_factor, 3)
    left_request.ik_request.pose_stamped.pose.position.z = round(-1*msg.left_wrist[1]*hip_scale_factor, 3)
    left_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.w = 1.0
    left_request.ik_request.avoid_collisions = True

    print("Scale factor", hip_scale_factor, hips_distance)
    print("Scaled wrist", msg.right_wrist[0]*hip_scale_factor, msg.right_wrist[1]*hip_scale_factor, msg.right_wrist[2]*hip_scale_factor)

    right_response : GetPositionIKResponse = False
    left_response : GetPositionIKResponse = False
    try:
        right_response = compute_ik(right_request)
        left_response = compute_ik(left_request)
    except ValueError: 
        print("ValueError!")

    print("response", right_response, left_response)
    if right_response.error_code.val == 1 and left_response.error_code.val == 1:
        angles_msg = Angles()
        angles_msg.right_arm = [x for x in right_response.solution.joint_state.position[-7:-3]] + [0]
        angles_msg.left_arm = [x for x in left_response.solution.joint_state.position[-14:-10]] + [0]
        pub.publish(angles_msg)
    else:
        print("No angles computed")

def app():
    sub = rospy.Subscriber('preprocessed', Landmarks, generate_angles)
    rospy.init_node('kinematics', anonymous=False)

    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass


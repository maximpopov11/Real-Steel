import sys
import rospy
from custom_msg.msg import Landmarks, Angles
from kin_util import timestamp
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

FUDGE_FACTOR = 0.7
ROBOT_HIP_METERS = 0.25*FUDGE_FACTOR
ROBOT_SHOULDER_TO_HIP = .27

def compute_distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**(1/2)

def generate_angles(msg):
    # Get the timestamp so we can time how long this takes to run
    before_ts = timestamp()

    # computation is done through a ROS service
    rospy.wait_for_service("compute_ik")
    
    # Assume the points are preprocessed and relative to the midpoint of the hips
    # Get a position position request and set it up for the right arm and hand
    # This should be updated to make one request for both left and right arms simultaneously.
    right_request = GetPositionIKRequest()
    right_request.ik_request.group_name = "right_arm"
    right_request.ik_request.ik_link_name = "right_rubber_hand"
    right_request.ik_request.pose_stamped.pose.position.x = msg.right_wrist[0]
    right_request.ik_request.pose_stamped.pose.position.y = msg.right_wrist[1]
    right_request.ik_request.pose_stamped.pose.position.z = msg.right_wrist[2]
    right_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    right_request.ik_request.pose_stamped.pose.orientation.w = 1.0
    right_request.ik_request.avoid_collisions = True

    # Repeat all for left arm
    left_request = GetPositionIKRequest()
    left_request.ik_request.group_name = "left_arm"
    left_request.ik_request.ik_link_name = "left_rubber_hand"
    left_request.ik_request.pose_stamped.pose.position.x = msg.left_wrist[0]
    left_request.ik_request.pose_stamped.pose.position.y = msg.left_wrist[1]
    left_request.ik_request.pose_stamped.pose.position.z = msg.left_wrist[2]
    left_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.y = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    left_request.ik_request.pose_stamped.pose.orientation.w = 1.0
    left_request.ik_request.avoid_collisions = True

    # Establish varialbes for the responses
    right_response : GetPositionIKResponse = False
    left_response : GetPositionIKResponse = False
    try:
        right_response = compute_ik(right_request)
        left_response = compute_ik(left_request)
    except ValueError: 
        print("ValueError!")
    lt = left_request.ik_request.pose_stamped.pose.position
    rt = right_request.ik_request.pose_stamped.pose.position

    # If the error code is 1 we successfully did IK
    if right_response.error_code.val == 1 and left_response.error_code.val == 1:
        angles_msg = Angles()
        ### SET Time???? ###
        # msg.header = Header(stamp=rospy.Time.now(), frame_id="your_frame_id") 
        # the returned position field is a 3-tuple; we need it to be a list. Also cropping out just the 4 angles we  want
        angles_msg.right_arm = [x for x in right_response.solution.joint_state.position[-7:-3]] + [0]
        angles_msg.left_arm = [x for x in left_response.solution.joint_state.position[-14:-10]] + [0]
        pub.publish(angles_msg)
        after_ts = timestamp()
       #rospy.loginfo(f"({after_ts - before_ts}ms): Angles Left: {angles_msg.left_arm} Right: {angles_msg.right_arm}")
        rospy.loginfo(f"ROBOT_COORD_SYS LT: ({lt.x}, {lt.y}, {lt.z}) RT: ({rt.x}, {rt.y}, {rt.z})")
        rospy.loginfo(f"CAMERA_COORD_SYS LT: ({lt.y}, {lt.z}, {lt.x}) RT: ({rt.y}, {rt.z}, {rt.x})")
    else:
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


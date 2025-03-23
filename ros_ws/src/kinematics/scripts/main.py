import rospy
from custom_msg.msg import Landmarks, Angles
from kin_util import timestamp

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)

def generate_angles(msg):
    """
    Through kinematic processes, take a message of landmarks and produce
    """
    print(timestamp(), msg)
    angles_msg = Angles()
    angles_msg.left_arm  = [0, 0, 0, 0, 0]
    angles_msg.right_arm = [0, 0, 0, 0, 0]
    pub.publish(angles_msg)

def app():
    sub = rospy.Subscriber('landmarks', Landmarks, generate_angles)
    rospy.init_node('kinematics', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
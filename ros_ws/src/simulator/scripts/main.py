import rospy
from custom_msg.msg import Angles
from sim_util import timestamp

def use_angles(msg):
    """
    Placeholder for the robot long-term. Use simulator or other systems to visualize the angle output.
    """
    print(timestamp(), msg)

def app():
    sub = rospy.Subscriber('robot_angles', Angles, use_angles)
    rospy.init_node('simulator', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
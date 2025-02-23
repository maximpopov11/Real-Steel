import rospy
from custom_msg.msg import arm

def display(msg):
    print("hello", msg)

def app():
    sub = rospy.Subscriber('arms', arm, display)

    rospy.init_node('POSE_Display', anonymous=True)
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
import rospy
from custom_msg.msg import arm

def run():
    pub = rospy.Publisher('left_arm', arm, queue_size=10)
    rospy.init_node('robot_camera', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = arm()
        msg.shoulder = [1.0, 2.0, 3.0]
        msg.elbow = [4.0, 5.0, 6.0]
        msg.wrist = [7.0, 8.0, 9.0]

        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
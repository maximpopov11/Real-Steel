import time
import sys
import csv
import rospy
from custom_msg.msg import Landmarks, Angles

pub = rospy.Publisher('robot_angles', Angles, queue_size=10)
WAIT_HERTZ = 10

def app(filereader):
    header = next(filereader)
    print(header)
    rospy.init_node('csv_publisher', anonymous=False)

    rate = rospy.Rate(WAIT_HERTZ)
    for row in filereader:
        angles_msg = Angles()
        angles_msg.left_arm = [float(x) for x in row[1:7]]
        angles_msg.right_arm = [float(x) for x in row[7:]]

        pub.publish(angles_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            print("err: needs csv filename")
        else:
            with open(sys.argv[1], 'r') as csv_file:
                filereader = csv.reader(csv_file)
                app(filereader)

    except rospy.ROSInterruptException:
        pass


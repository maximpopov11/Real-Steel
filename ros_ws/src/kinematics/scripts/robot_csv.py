#!/usr/bin/env python

import rospy
import csv
from custom_msg.msg import Angles

class CsvWriterNode:
    def __init__(self, output_filename):
        self.output_filename = output_filename
        self.file = open(self.output_filename, 'w')
        self.writer = csv.writer(self.file)
        # Write CSV headers
        headers = ['timestamp']
        headers += [f'left_joint_{i}' for i in range(5)]
        headers += [f'right_joint_{i}' for i in range(5)]
        self.writer.writerow(headers)
        rospy.loginfo(f"CSV file '{self.output_filename}' created with headers.")
        
        # Subscribe to robot_angles topic
        self.subscriber = rospy.Subscriber('robot_angles', Angles, self.callback)
        rospy.loginfo("Subscribed to 'robot_angles' topic.")

    def callback(self, msg):
        rospy.loginfo("Received angles message.")
        rospy.loginfo(f"Message content: {msg}\n")
        # Check if the message is valid
        if not isinstance(msg, Angles):
            rospy.logwarn("Received invalid message type.")
            return
        # Extract timestamp from the message header
        # timestamp = msg.header.stamp.to_sec()
        timestamp = rospy.get_time()
        # Extract joint angles
        left_arm = list(msg.left_arm)
        right_arm = list(msg.right_arm)
        # Prepare the CSV row
        row = [timestamp] + left_arm + right_arm
        # Write to CSV
        self.writer.writerow(row)
        rospy.loginfo(f"Recorded angles at {timestamp}: left={left_arm}, right={right_arm}")

    def shutdown_hook(self):
        self.file.close()
        rospy.loginfo(f"CSV file '{self.output_filename}' closed.")

def main():
    rospy.init_node('robot_csv')
    # Get the output filename parameter, defaulting to 'angles.csv'
    output_filename = rospy.get_param('~output_file', 'angles.csv')
    rospy.loginfo(f"Output filename set to: {output_filename}")
    
    csv_node = CsvWriterNode(output_filename)
    rospy.on_shutdown(csv_node.shutdown_hook)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
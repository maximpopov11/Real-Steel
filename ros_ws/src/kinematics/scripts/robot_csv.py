#!/usr/bin/env python
"""
Need to get speed between angles. Don't need distance.
"""
import rospy
import csv
import math
from custom_msg.msg import Angles

class CsvWriterNode:
    def __init__(self, output_filename):
        self.output_filename = output_filename
        self.file = open(self.output_filename, 'w')
        self.writer = csv.writer(self.file)
        headers = ['timestamp']
        headers += [f'left_joint_{i}' for i in range(5)]
        headers += [f'right_joint_{i}' for i in range(5)]
        self.writer.writerow(headers)
        rospy.loginfo(f"CSV file '{self.output_filename}' created with headers.")
        
        self.subscriber = rospy.Subscriber('robot_angles', Angles, self.callback)
        rospy.loginfo("Subscribed to 'robot_angles' topic.")

        self.time_count = 0.0
        self.last_angles = None  # To store the last written angles

    def interpolate_points(self, start_angles, end_angles, num_steps):
        if num_steps <= 0:
            return []
        interpolated = []
        for step in range(1, num_steps + 1):
            alpha = step / num_steps
            interp = [s + alpha * (e - s) for s, e in zip(start_angles, end_angles)]
            interpolated.append(interp)
        return interpolated

    def callback(self, msg):
        rospy.loginfo("Received angles message.")
        if not isinstance(msg, Angles):
            rospy.logwarn("Received invalid message type.")
            return

        left_arm = list(msg.left_arm)
        right_arm = list(msg.right_arm)
        current_angles = left_arm + right_arm

        if self.last_angles is None:
            # First message, write directly
            timestamp = f"{self.time_count:.3f}"
            row = [timestamp] + left_arm + right_arm
            self.writer.writerow(row)
            self.last_angles = current_angles
            self.time_count += 0.1
            rospy.loginfo(f"Recorded initial angles at {timestamp}")
            return

        # Calculate angular speed for each joint (rad/s)
        time_diff = 0.1  # Fixed time step between callbacks
        speed_per_joint = [
            abs(curr - last) / time_diff
            for curr, last in zip(current_angles, self.last_angles)
        ]

        # Find the maximum speed across all joints
        max_speed = max(speed_per_joint)

        if max_speed > 34.0:  # Threshold is 34 rad/s
            # Calculate required steps to stay under 34 rad/s
            required_steps = math.ceil(max_speed / 34.0)
            rospy.loginfo(f"Max speed {max_speed:.2f} rad/s exceeds threshold. Interpolating {required_steps} steps.")
            interpolated = self.interpolate_points(self.last_angles, current_angles, required_steps)
            for angles in interpolated:
                left = angles[:5]
                right = angles[5:]
                timestamp = f"{self.time_count:.3f}"
                row = [timestamp] + left + right
                self.writer.writerow(row)
                rospy.loginfo(f"Recorded interpolated angles at {timestamp}")
                self.time_count += 0.1
            self.last_angles = current_angles
        else:
            # Write current angles
            timestamp = f"{self.time_count:.3f}"
            row = [timestamp] + left_arm + right_arm
            self.writer.writerow(row)
            rospy.loginfo(f"Recorded angles at {timestamp}")
            self.time_count += 0.1
            self.last_angles = current_angles
            
    def shutdown_hook(self):
        self.file.close()
        rospy.loginfo(f"CSV file '{self.output_filename}' closed.")

def main():
    rospy.init_node('robot_csv')
    output_filename = rospy.get_param('~output_file', 'angles.csv')
    csv_node = CsvWriterNode(output_filename)
    rospy.on_shutdown(csv_node.shutdown_hook)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
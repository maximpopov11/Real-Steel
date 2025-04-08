#!/usr/bin/env python

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

        # Calculate Euclidean distance between current and last angles
        distance = 0.0
        for i in range(len(current_angles)):
            diff = current_angles[i] - self.last_angles[i]
            distance += diff ** 2
        distance = math.sqrt(distance)

        time_diff = 0.1  # Fixed time step between callbacks
        speed = distance / time_diff

        if speed > 0.5:
            # Need to interpolate
            required_time = distance / 0.5
            num_steps = math.ceil(required_time / time_diff)
            rospy.loginfo(f"Speed {speed:.2f} m/s exceeds threshold. Interpolating {num_steps} steps.")
            interpolated = self.interpolate_points(self.last_angles, current_angles, num_steps)
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
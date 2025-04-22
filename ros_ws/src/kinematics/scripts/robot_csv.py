#!/usr/bin/env python

import os
import rospy
import csv
import math
from custom_msg.msg import Angles
SPEED_THRESHOLD = 2.0  # rad/s



left_arm_joint_names = [
    "left_shoulder_pitch_joint", 
    "left_shoulder_roll_joint", 
    "left_shoulder_yaw_joint", 
    "left_elbow_joint"
]

right_arm_joint_names = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint"
]

class CsvWriterNode:
    def __init__(self, output_filename):
        self.SPEED_THRESHOLD = SPEED_THRESHOLD  # rad/s
        self.output_filename = output_filename
        output_dir = os.path.dirname(self.output_filename)
        if output_dir:  # Only create if path contains directory
            os.makedirs(output_dir, exist_ok=True)
        self.file = open(self.output_filename, 'w')
        self.writer = csv.writer(self.file)
        headers = ['timestamp']
        headers += left_arm_joint_names
        headers += right_arm_joint_names
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
            # Calculate interpolation ratio (0 = start, 1 = end)
            alpha = step / num_steps  # Ranges from 1/num_steps to 1.0
            # Linear interpolation for each joint: 
            # angle = start + (end - start) * progress_ratio
            interp = [s + alpha * (e - s) for s, e in zip(start_angles, end_angles)]
            # Store this interpolated step
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

        if max_speed > self.SPEED_THRESHOLD:  # Threshold is 35 rad/s
            # Calculate required steps to stay under 35 rad/s
            required_steps = math.ceil(max_speed / self.SPEED_THRESHOLD)
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
    # Get the output filename from the parameter server. Default: 'angles.csv'
    output_filename = rospy.get_param('~output_file', 'angles.csv')
    csv_node = CsvWriterNode(output_filename)
    rospy.on_shutdown(csv_node.shutdown_hook)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
import os
import rospy
import rospkg
import csv
import math
from custom_msg.msg import Angles
from kin_util import SPEED_THRESHOLD, interpolate_points
from joint_names import left_arm_joint_names, right_arm_joint_names
from validity_checker import StateValidityChecker

validity_checker = StateValidityChecker()

def interpolated_angles_valid(angles):
    validity_checker.setJointStates(angles)
    return validity_checker.getStateValidity('left_arm') and validity_checker.getStateValidity('right_arm')
    

class CsvWriterNode:
    def __init__(self, output_filename):
        self.SPEED_THRESHOLD = SPEED_THRESHOLD  # rad/s
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('kinematics')
        
        # Create default path if empty
        if not output_filename:
            output_filename = os.path.join(pkg_path, "output", "angles.csv")
        
        # Resolve absolute path
        self.output_filename = os.path.abspath(output_filename)
        
        # Create directory if needed
        output_dir = os.path.dirname(self.output_filename)
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

    def callback(self, msg):
        rospy.loginfo("Received angles message.")
        if not isinstance(msg, Angles):
            rospy.logwarn("Received invalid message type.")
            return

        left_arm = list(msg.left_arm)
        right_arm = list(msg.right_arm)
        current_angles = left_arm + right_arm

        if self.last_angles is None:
            timestamp = f"{self.time_count:.3f}"
            row = [timestamp] + left_arm + right_arm # uncomment this normally (for arm with 6 dof)
            # row = [timestamp] + left_arm + [0] + right_arm + [0] # comment if not using dof of 6
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

        max_speed = max(speed_per_joint)

        if max_speed > SPEED_THRESHOLD:
            required_steps = math.ceil(max_speed / SPEED_THRESHOLD)
            rospy.loginfo(f"Max speed {max_speed:.2f} rad/s exceeds threshold. Interpolating {required_steps} steps.")
            interpolated = interpolate_points(self.last_angles, current_angles, required_steps)
            for angles in interpolated:
                left = angles[:5]
                right = angles[5:]

                if not interpolated_angles_valid(angles):
                    rospy.logerr(f"Interpolated angles are invalid:\n{angles}")
                    break

                timestamp = f"{self.time_count:.3f}"
                row = [timestamp] + left + right # uncomment this normally (for arm with 6 dof)
                # row = [timestamp] + left + [0] + right + [0] # comment if not using dof of 6
                self.writer.writerow(row)
                rospy.loginfo(f"Recorded interpolated angles at {timestamp}")
                self.time_count += 0.1
            self.last_angles = current_angles
        else:
            # Write current angles
            timestamp = f"{self.time_count:.3f}"
            row = [timestamp] + left_arm + right_arm # uncomment this normally (for arm with 6 dof)
            # row = [timestamp] + left_arm + [0] + right_arm + [0] # comment if not using dof of 6
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
    output_filename = rospy.get_param('~output_file', '')
    csv_node = CsvWriterNode(output_filename)
    rospy.on_shutdown(csv_node.shutdown_hook)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

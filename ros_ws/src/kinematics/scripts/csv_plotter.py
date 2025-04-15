#!/usr/bin/env python

import rospy
import csv
import matplotlib.pyplot as plt
import os
import sys

class JointAnglePlotter:
    def __init__(self, input_filename):
        self.input_filename = os.path.abspath(input_filename)
        rospy.loginfo(f"Input file: {self.input_filename}")
        
        if not os.path.exists(self.input_filename):
            rospy.logerr(f"File not found: {self.input_filename}")
            sys.exit(1)
            
        self.timestamps = []
        self.joint_angles = {
            'left': {i: {'values': [], 'start': 0.0} for i in range(5)},
            'right': {i: {'values': [], 'start': 0.0} for i in range(5)}
        }
        
        self.read_csv()
        self.plot_individual_joints()

    def read_csv(self):
        try:
            with open(self.input_filename, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)
                rospy.loginfo(f"CSV headers: {headers}")
                
                for row_idx, row in enumerate(reader):
                    # Convert timestamp to relative time
                    current_time = float(row[0]) if row_idx == 0 else float(row[0]) - float(self.timestamps[0])
                    self.timestamps.append(current_time)
                    
                    # Process left arm joints
                    for joint in range(5):
                        angle = float(row[1+joint])
                        self.joint_angles['left'][joint]['values'].append(angle)
                        if row_idx == 0:
                            self.joint_angles['left'][joint]['start'] = angle
                    
                    # Process right arm joints
                    for joint in range(5):
                        angle = float(row[6+joint])
                        self.joint_angles['right'][joint]['values'].append(angle)
                        if row_idx == 0:
                            self.joint_angles['right'][joint]['start'] = angle
            
            rospy.loginfo(f"Processed {len(self.timestamps)} data points")
            
        except Exception as e:
            rospy.logerr(f"CSV read error: {str(e)}")
            raise

    def plot_individual_joints(self):
        output_dir = os.path.splitext(self.input_filename)[0] + "_plots"
        os.makedirs(output_dir, exist_ok=True)

        # Plot left arm joints
        for joint in range(5):
            fig, ax = plt.subplots(figsize=(10, 5))
            data = self.joint_angles['left'][joint]
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='blue')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            
            ax.set_title(f'Left Arm Joint {joint} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()
            
            output_path = os.path.join(output_dir, f'left_joint_{joint}.png')
            plt.savefig(output_path)
            plt.close()
            rospy.loginfo(f"Saved plot: {output_path}")

        # Plot right arm joints
        for joint in range(5):
            fig, ax = plt.subplots(figsize=(10, 5))
            data = self.joint_angles['right'][joint]
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='green')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            
            ax.set_title(f'Right Arm Joint {joint} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()
            
            output_path = os.path.join(output_dir, f'right_joint_{joint}.png')
            plt.savefig(output_path)
            plt.close()
            rospy.loginfo(f"Saved plot: {output_path}")

def main():
    rospy.init_node('joint_angle_visualizer')
    
    # Get input file parameter with validation
    input_file = rospy.get_param('~input_file', 'angles.csv')
    rospy.loginfo(f"Received input file parameter: {input_file}")
    
    # Convert to absolute path
    input_file = os.path.abspath(input_file)
    
    try:
        plotter = JointAnglePlotter(input_file)
    except Exception as e:
        rospy.logerr(f"Visualization failed: {str(e)}")
        sys.exit(1)
    
    rospy.signal_shutdown("Visualization complete")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
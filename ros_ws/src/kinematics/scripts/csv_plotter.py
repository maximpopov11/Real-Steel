#!/usr/bin/env python

import rospy
import csv
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class KinematicsPlotter:
    def __init__(self, input_filename):
        self.input_filename = input_filename
        self.timestamps = []
        self.joint_positions = {'left': [], 'right': []}  # Stores [joint][axis][time]
        
        self.read_csv()
        self.calculate_kinematics()
        self.plot_trajectories()

    def simple_forward_kinematics(self, angles):
        """Simplified FK assuming consecutive XY rotations with unit links"""
        x, y, z = 0, 0, 0
        positions = []
        for i, angle in enumerate(angles):
            x += math.cos(sum(angles[:i+1]))
            y += math.sin(sum(angles[:i+1]))
            z += 0.2 * math.sin(angle)  # Add vertical component
            positions.append((x, y, z))
        return positions

    def read_csv(self):
        try:
            with open(self.input_filename, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)
                
                for row in reader:
                    self.timestamps.append(float(row[0]))
                    
                    # Process left arm
                    left_angles = [float(row[i]) for i in range(1, 6)]
                    self.joint_positions['left'].append(
                        self.simple_forward_kinematics(left_angles)
                    )
                    
                    # Process right arm
                    right_angles = [float(row[i]) for i in range(6, 11)]
                    self.joint_positions['right'].append(
                        self.simple_forward_kinematics(right_angles)
                    )
            
            rospy.loginfo(f"Processed {len(self.timestamps)} data points")
            
        except Exception as e:
            rospy.logerr(f"CSV read error: {str(e)}")
            raise

    def calculate_kinematics(self):
        # Reorganize data: [joint][axis][time_index]
        self.organized_data = {'left': {}, 'right': {}}
        
        for arm in ['left', 'right']:
            num_joints = len(self.joint_positions[arm][0])
            for joint_idx in range(num_joints):
                self.organized_data[arm][joint_idx] = {
                    'x': [],
                    'y': [],
                    'z': []
                }
                
                for time_idx in range(len(self.timestamps)):
                    pos = self.joint_positions[arm][time_idx][joint_idx]
                    self.organized_data[arm][joint_idx]['x'].append(pos[0])
                    self.organized_data[arm][joint_idx]['y'].append(pos[1])
                    self.organized_data[arm][joint_idx]['z'].append(pos[2])

    def plot_trajectories(self):
        fig = plt.figure(figsize=(20, 15))
        
        # Plot left arm
        for joint in range(5):
            ax = fig.add_subplot(2, 5, joint+1, projection='3d')
            x = self.organized_data['left'][joint]['x']
            y = self.organized_data['left'][joint]['y']
            z = self.organized_data['left'][joint]['z']
            ax.plot(x, y, z)
            ax.set_title(f'Left Joint {joint} Trajectory')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
        
        # Plot right arm
        for joint in range(5):
            ax = fig.add_subplot(2, 5, joint+6, projection='3d')
            x = self.organized_data['right'][joint]['x']
            y = self.organized_data['right'][joint]['y']
            z = self.organized_data['right'][joint]['z']
            ax.plot(x, y, z)
            ax.set_title(f'Right Joint {joint} Trajectory')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')

        plt.tight_layout()
        
        # Save and show
        output_file = self.input_filename.replace('.csv', '_3d_trajectories.png')
        plt.savefig(output_file)
        rospy.loginfo(f"Saved 3D plot to {output_file}")
        plt.show()

def main():
    rospy.init_node('kinematics_visualizer')
    input_file = rospy.get_param('~input_file', 'angles.csv')
    
    try:
        plotter = KinematicsPlotter(input_file)
    except Exception as e:
        rospy.logerr(f"Visualization failed: {str(e)}")
    
    rospy.signal_shutdown("Visualization complete")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
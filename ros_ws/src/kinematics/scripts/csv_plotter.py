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
        self.MAX_SPEED_THRESHOLD = 35.0  # rad/s threshold from robot_csv.py
        self.read_csv()
        self.plot_joint_data()

    def read_csv(self):
        try:
            with open(self.input_filename, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)
                rospy.loginfo(f"CSV headers: {headers}")
                
                for row_idx, row in enumerate(reader):
                    current_time = float(row[0]) 
                    if row_idx == 0:
                        self.timestamps.append(0.0)
                    else:
                        self.timestamps.append(current_time - self.timestamps[0])
                    
                    for joint in range(5):
                        angle = float(row[1+joint])
                        self.joint_angles['left'][joint]['values'].append(angle)
                        if row_idx == 0:
                            self.joint_angles['left'][joint]['start'] = angle
                    
                    for joint in range(5):
                        angle = float(row[6+joint])
                        self.joint_angles['right'][joint]['values'].append(angle)
                        if row_idx == 0:
                            self.joint_angles['right'][joint]['start'] = angle
            
            rospy.loginfo(f"Processed {len(self.timestamps)} data points")
            
        except Exception as e:
            rospy.logerr(f"CSV read error: {str(e)}")
            raise

    def _plot_speed(self, joint_type, joint_num, values, output_dir):
        """Helper function to plot speed for a single joint"""
        times = self.timestamps
        speeds = []
        speed_times = []
        
        if len(values) < 2:
            rospy.logwarn(f"Not enough data to compute speed for {joint_type} joint {joint_num}")
            return
        
        for i in range(1, len(values)):
            delta_t = times[i] - times[i-1]
            if delta_t <= 0:
                continue
            speed = (values[i] - values[i-1]) / delta_t #speed / time
            speeds.append(speed)
            speed_times.append(times[i])

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(speed_times, speeds, color='orange', label='Joint Speed')
        ax.axhline(y=self.MAX_SPEED_THRESHOLD, color='red', linestyle='--', 
                  label=f'Max Speed Threshold ({self.MAX_SPEED_THRESHOLD} rad/s)')
        ax.set_title(f'{joint_type.capitalize()} Arm Joint {joint_num} Speed vs Time')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (rad/s)')
        ax.grid(True)
        ax.legend()
        
        output_path = os.path.join(output_dir, f'{joint_type}_joint_{joint_num}_speed.png')
        plt.savefig(output_path)
        plt.close()
        rospy.loginfo(f"Saved speed plot: {output_path}")

    def plot_joint_data(self):
        output_dir = os.path.splitext(self.input_filename)[0] + "_plots"
        os.makedirs(output_dir, exist_ok=True)

        # Plot left arm joints
        for joint in range(5):
            # Plot angles
            fig, ax = plt.subplots(figsize=(10, 5))
            data = self.joint_angles['left'][joint]
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='blue')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            ax.set_title(f'Left Arm Joint {joint} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()
            output_path = os.path.join(output_dir, f'left_joint_{joint}_angle.png')
            plt.savefig(output_path)
            plt.close()
            
            # Plot speeds
            self._plot_speed('left', joint, data['values'], output_dir)

        # Plot right arm joints
        for joint in range(5):
            # Plot angles
            fig, ax = plt.subplots(figsize=(10, 5))
            data = self.joint_angles['right'][joint]
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='green')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            ax.set_title(f'Right Arm Joint {joint} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()
            output_path = os.path.join(output_dir, f'right_joint_{joint}_angle.png')
            plt.savefig(output_path)
            plt.close()
            
            # Plot speeds
            self._plot_speed('right', joint, data['values'], output_dir)

def main():
    rospy.init_node('joint_angle_visualizer')
    input_file = rospy.get_param('~input_file', 'angles.csv')
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
#!/usr/bin/env python

import rospy
import csv
import matplotlib.pyplot as plt
import os
import sys
from kin_util import SPEED_THRESHOLD

class JointAnglePlotter:
    def __init__(self, input_filename):
        self.input_filename = os.path.abspath(input_filename)
        rospy.loginfo(f"Input file: {self.input_filename}")

        if not os.path.exists(self.input_filename):
            rospy.logerr(f"File not found: {self.input_filename}")
            sys.exit(1)

        self.timestamps = []
        self.joint_angles = {'left': [], 'right': []}
        self.joint_names = {'left': [], 'right': []}
        self.MAX_SPEED_THRESHOLD = 1.0
        self.read_csv()
        self.plot_joint_data()

    def read_csv(self):
        try:
            with open(self.input_filename, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)
                rospy.loginfo(f"CSV headers: {headers}")

                # Extract joint names from headers
                self.joint_names['left'] = headers[1:7]  # First 6 after timestamp
                self.joint_names['right'] = headers[7:11] # Next 6

                # Initialize data structures
                self.joint_angles['left'] = [{'values': [], 'start': 0.0} for _ in range(5)]
                self.joint_angles['right'] = [{'values': [], 'start': 0.0} for _ in range(5)]

                for row_idx, row in enumerate(reader):
                    current_time = float(row[0])
                    if row_idx == 0:
                        self.timestamps.append(0.0)
                    else:
                        self.timestamps.append(current_time - self.timestamps[0])

                    # Read left joints
                    for i in range(5):
                        val = float(row[1+i])
                        self.joint_angles['left'][i]['values'].append(val)
                        if row_idx == 0:
                            self.joint_angles['left'][i]['start'] = val

                    # Read right joints
                    for i in range(5):
                        val = float(row[6+i])
                        self.joint_angles['right'][i]['values'].append(val)
                        if row_idx == 0:
                            self.joint_angles['right'][i]['start'] = val

            rospy.loginfo(f"Processed {len(self.timestamps)} data points")

        except Exception as e:
            rospy.logerr(f"CSV read error: {str(e)}")
            raise

    def _plot_speed(self, joint_name, values, output_dir):
        """Helper function to plot speed for a single joint"""
        times = self.timestamps
        speeds = []
        speed_times = []

        if len(values) < 2:
            rospy.logwarn(f"Not enough data to compute speed for {joint_name}")
            return

        for i in range(1, len(values)):
            delta_t = times[i] - times[i-1]
            if delta_t <= 0:
                continue
            speed = (values[i] - values[i-1]) / delta_t
            speeds.append(speed)
            speed_times.append(times[i])

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(speed_times, speeds, color='orange', label='Joint Speed')
        ax.axhline(y=self.SPEED_THRESHOLD, color='red', linestyle='--', 
                  label=f'Max Threshold ({self.SPEED_THRESHOLD} rad/s)')
        ax.set_title(f'{joint_name} Speed vs Time')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (rad/s)')
        ax.grid(True)
        ax.legend()

        # Sanitize filename
        safe_name = "".join([c if c.isalnum() else "_" for c in joint_name])
        output_path = os.path.join(output_dir, f'{safe_name}_speed.png')
        plt.savefig(output_path)
        plt.close()
        rospy.loginfo(f"Saved speed plot: {output_path}")

    def plot_joint_data(self):
        output_dir = os.path.splitext(self.input_filename)[0] + "_plots"
        os.makedirs(output_dir, exist_ok=True)

        # Plot left arm joints
        for idx, joint_name in enumerate(self.joint_names['left']):
            data = self.joint_angles['left'][idx]

            # Plot angles
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='blue')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            ax.set_title(f'{joint_name} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()

            # Sanitize filename
            safe_name = "".join([c if c.isalnum() else "_" for c in joint_name])
            output_path = os.path.join(output_dir, f'{safe_name}_angle.png')
            plt.savefig(output_path)
            plt.close()

            # Plot speeds
            self._plot_speed(joint_name, data['values'], output_dir)

        # Plot right arm joints
        for idx, joint_name in enumerate(self.joint_names['right']):
            data = self.joint_angles['right'][idx]

            # Plot angles
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.plot(self.timestamps, data['values'], label='Joint Angle', color='green')
            ax.axhline(y=data['start'], color='red', linestyle='--', label='Start Position')
            ax.set_title(f'{joint_name} Angle vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angle (rad)')
            ax.grid(True)
            ax.legend()

            # Sanitize filename
            safe_name = "".join([c if c.isalnum() else "_" for c in joint_name])
            output_path = os.path.join(output_dir, f'{safe_name}_angle.png')
            plt.savefig(output_path)
            plt.close()

            # Plot speeds
            self._plot_speed(joint_name, data['values'], output_dir)

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

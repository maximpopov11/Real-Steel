import rospy
from custom_msg.msg import Landmarks
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

plt.ion()  # Turn on interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

def on_close(event):
    rospy.signal_shutdown("Matplotlib window closing")

fig.canvas.mpl_connect('close_event', on_close)


# Label the axes
# Adjust the view so we mostly see the XY plane
ax.view_init(elev=90, azim=90, roll=180)  # Top-down view looking at the XY plane

instants_received = [0, 0]
list_of_x_coord_lists, list_of_y_coord_lists, list_of_z_coord_lists = [[], []], [[], []], [[], []]
def get_plotting_callback(points_idx=0):
    def plot_landmarks_in_3d(msg):
        global instants_received, list_of_z_coord_lists, list_of_x_coord_lists, list_of_y_coord_lists
        # Set the moment we got these for the while loop later to use
        instants_received[points_idx] = time.time()
        # Extract x, y, z coords into arrays for matplotlib to graph
        list_of_x_coord_lists[points_idx] = [
            msg.left_hip[0],
            msg.right_hip[0],
            msg.left_shoulder[0],
            msg.right_shoulder[0],
            msg.left_elbow[0],
            msg.right_elbow[0],
            msg.left_wrist[0],
            msg.right_wrist[0],
            msg.left_pinky[0],
            msg.right_pinky[0],
            msg.left_index[0],
            msg.right_index[0],
            msg.left_thumb[0],
            msg.right_thumb[0],
            msg.nose[0]
        ]
        list_of_y_coord_lists[points_idx] = [
            msg.left_hip[1],
            msg.right_hip[1],
            msg.left_shoulder[1],
            msg.right_shoulder[1],
            msg.left_elbow[1],
            msg.right_elbow[1],
            msg.left_wrist[1],
            msg.right_wrist[1],
            msg.left_pinky[1],
            msg.right_pinky[1],
            msg.left_index[1],
            msg.right_index[1],
            msg.left_thumb[1],
            msg.right_thumb[1],
            msg.nose[1]
        ]
        list_of_z_coord_lists[points_idx] = [
            msg.left_hip[2],
            msg.right_hip[2],
            msg.left_shoulder[2],
            msg.right_shoulder[2],
            msg.left_elbow[2],
            msg.right_elbow[2],
            msg.left_wrist[2],
            msg.right_wrist[2],
            msg.left_pinky[2],
            msg.right_pinky[2],
            msg.left_index[2],
            msg.right_index[2],
            msg.left_thumb[2],
            msg.right_thumb[2],
            msg.nose[2]
        ]
    return plot_landmarks_in_3d



def app():
    sub = rospy.Subscriber('landmarks', Landmarks, get_plotting_callback(0))
    sub = rospy.Subscriber('preprocessed', Landmarks, get_plotting_callback(1))
    rospy.init_node('POSE_Display', anonymous=True)
    original_point_colors = ['red', 'blue', 'red', 'blue', 'red', 'blue', 'red', 'blue', 'black', 'black', 'black', 'black', 'green', 'green', 'purple']
    all_orange_colors = ['orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange']
    all_blue_colors = ['blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue']
    orange_and_blue_color_sets = all_orange_colors + all_blue_colors
    while not rospy.is_shutdown():
        # Ensure the bounds of the graph are drawn correctly
        ax.set_xlim([-640, 640])
        ax.set_ylim([0, 1280])
        ax.set_zlim([4, 0])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")

        plt.draw()

        # Small pause to allow the figure to update
        plt.pause(0.001)  

        # If the last frame was more than a tenth of a second ago, skip redrawing it
        if time.time() - instants_received[0] > .1 and time.time() - instants_received[1] > .1:
            continue

        all_x_points = list_of_x_coord_lists[0] + list_of_x_coord_lists[1]
        all_y_points = list_of_y_coord_lists[0] + list_of_y_coord_lists[1]
        all_z_points = list_of_z_coord_lists[0] + list_of_z_coord_lists[1]
        color_set = all_orange_colors
        if len(all_x_points) > 15:
            color_set = orange_and_blue_color_sets
        elif len(list_of_x_coord_lists[1]) > 0:
            color_set = all_blue_colors

        ax.clear()  # Clear previous points
        ax.scatter(all_x_points, all_y_points, all_z_points, color=color_set)

    print("exited loop")
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
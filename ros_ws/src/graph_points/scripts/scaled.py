import rospy
from custom_msg.msg import Landmarks
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time


def on_close(event):
    rospy.signal_shutdown("Matplotlib window closing")


plt.ion()
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close)
ax = fig.add_subplot(111, projection="3d")
ax.view_init(elev=90, azim=90, roll=180)

instants_received = 0
list_of_x_coord, list_of_y_coord, list_of_z_coord = [], [], []


def get_plotting_callback(msg):
    global instants_received, list_of_z_coord_lists, list_of_x_coord_lists, list_of_y_coord_lists
    # Set the moment we got these for the while loop later to use
    instants_received = time.time()
    # Extract x, y, z coords into arrays for matplotlib to graph
    list_of_x_coord = [
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
    list_of_y_coord = [
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
    list_of_z_coord = [
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
    return


def app():
    sub = rospy.Subscriber('scaled', Landmarks, get_plotting_callback)
    rospy.init_node('POSE_Display_Scaled', anonymous=True)
    all_green_colors = ['green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green', 'green']
    while not rospy.is_shutdown():
        ax.set_xlim([-1, 1])
        ax.set_ylim([0, 1.5])
        ax.set_zlim([2, -2])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")
        ax.set_box_aspect([5, 3.75, 4])

        plt.draw()

        plt.pause(0.001)  

        # If the last frame was more than a tenth of a second ago, skip redrawing it
        if time.time() - instants_received > .1 and time.time() - instants_received[1] > .1:
            continue

        ax.clear()

        # Transpose from robot coordinate system to the "normal" one
        ax.scatter(list_of_z_coord, list_of_x_coord, list_of_y_coord, color=all_green_colors)

    print("exited loop")
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

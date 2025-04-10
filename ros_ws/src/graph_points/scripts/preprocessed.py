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
ax.view_init(elev=90, azim=90, roll=180)  # Top-down view looking at the XY plane

instants_received = 0
list_of_x_coord, list_of_y_coord, list_of_z_coord = [], [], []


def plotting_callback(msg):
    global instants_received, list_of_z_coord, list_of_x_coord, list_of_y_coord
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
    sub = rospy.Subscriber('preprocessed', Landmarks, plotting_callback)
    rospy.init_node('POSE_Display_Preprocessed', anonymous=True)
    all_blue_colors = ['blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue', 'blue']
    while not rospy.is_shutdown():
        ax.set_xlim([-640, 640])
        ax.set_ylim([0, 480])
        ax.set_zlim([4, 0])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")
        ax.set_box_aspect([16, 6, 1])

        plt.draw()

        plt.pause(0.001)  

        # If the last frame was more than a tenth of a second ago, skip redrawing it
        if time.time() - instants_received > .1 and time.time() - instants_received > .1:
            continue

        ax.clear()
        ax.scatter(list_of_x_coord, list_of_y_coord, list_of_z_coord, color=all_blue_colors)

    print("exited loop")
    

if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

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

instant_received = 0
list_of_x_coord, list_of_y_coord, list_of_z_coord = [], [], []


def plotting_callback(msg):
    global instant_received, list_of_z_coord, list_of_x_coord, list_of_y_coord
    # Set the moment we got these for the while loop later to use
    instant_received = time.time()
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
   ]
    return 


def app():
    sub = rospy.Subscriber('landmarks', Landmarks, plotting_callback)
    rospy.init_node('POSE_Display_Raw_Landmarks', anonymous=True)
    all_orange_colors = ['orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'orange']
    while not rospy.is_shutdown():
        ax.set_xlim([0, 640])
        ax.set_ylim([0, 480])
        ax.set_zlim([4, 0])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")
        ax.set_box_aspect([4, 3, 2])

        plt.draw()

        plt.pause(0.001)  

        # If the last frame was more than a tenth of a second ago, skip redrawing it
        if time.time() - instant_received > .1 and time.time() - instant_received > .1:
            continue

        ax.clear()
        ax.scatter(list_of_x_coord, list_of_y_coord, list_of_z_coord, color=all_orange_colors)

    print("exited loop")
    

if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass

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
ax.view_init(elev=90, azim=90)  # Top-down view looking at the XY plane

instant_received = 0
x_coords, y_coords, z_coords = [], [], []
def plot_landmarks_in_3d(msg):
    global x_coords, y_coords, z_coords, instant_received
    # Set the moment we got these for the while loop later to use
    instant_received = time.time()
    # Extract x, y, z coords into arrays for matplotlib to graph
    x_coords = [
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
    y_coords = [
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
    z_coords = [
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

def app():
    sub = rospy.Subscriber('landmarks', Landmarks, plot_landmarks_in_3d)
    rospy.init_node('POSE_Display', anonymous=True)

    while not rospy.is_shutdown():
        # Ensure the bounds of the graph are drawn correctly
        ax.set_xlim([0, 640])
        ax.set_ylim([0, 480])
        ax.set_zlim([4, 0])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")

        plt.draw()

        # Small pause to allow the figure to update
        plt.pause(0.001)  

        # If the last frame was more than a tenth of a second ago, skip redrawing it
        if time.time() - instant_received > .1:
            continue

        ax.clear()  # Clear previous points
        ax.scatter(x_coords, y_coords, z_coords, color=['red', 'blue', 'red', 'blue', 'red', 'blue', 'red', 'blue', 'black', 'black', 'black', 'black', 'green', 'green', 'purple'])
        rospy.loginfo(f"Left Wrist: ({x_coords[6]}, {y_coords[6]}, {z_coords[6]})")   
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
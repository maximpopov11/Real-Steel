import rospy
from custom_msg.msg import arm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

plt.ion()  # Turn on interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Label the axes
# Adjust the view so we mostly see the XY plane
ax.view_init(elev=90, azim=90)  # Top-down view looking at the XY plane

instant_received = 0
x_coords, y_coords, z_coords = [], [], []
def plot_landmarks_in_3d(msg):
    #print("msg received!", msg)
    #return
    #landmarks = landmark_frame[0]
    # print(landmarks)

    #ax.clear()  # Clear previous points

    # Extract the 3D coordinates (x, y, z)
    #x_coords = [landmark.x for landmark in landmarks]  # Left/Right
    #y_coords = [landmark.y for landmark in landmarks]  # Up/Down
    #z_coords = [landmark.z for landmark in landmarks]  # Forward/Backward
    global x_coords, y_coords, z_coords, instant_received
    instant_received = time.time()
    x_coords = [
        msg.left_shoulder[0],
        msg.left_elbow[0],
        msg.left_wrist[0],
        msg.right_shoulder[0],
        msg.right_elbow[0],
        msg.right_wrist[0]
    ]
    y_coords = [
        msg.left_shoulder[1],
        msg.left_elbow[1],
        msg.left_wrist[1],
        msg.right_shoulder[1],
        msg.right_elbow[1],
        msg.right_wrist[1]
    ]
    z_coords = [
        msg.left_shoulder[2],
        msg.left_elbow[2],
        msg.left_wrist[2],
        msg.right_shoulder[2],
        msg.right_elbow[2],
        msg.right_wrist[2]
    ]

    #ax.scatter(x_coords, y_coords, z_coords)
    #plt.draw()
    #plt.pause(0.1)  # Small pause to allow the figure to update

#    print(f"Found {len(landmark_frames) - skipped} / {len(landmark_frames)}")



def display(msg):
    print("hello", msg)

def app():
    sub = rospy.Subscriber('arms', arm, plot_landmarks_in_3d)
    rospy.init_node('POSE_Display', anonymous=True)

    while True:
        #print("xcoords", x_coords)
        ax.set_xlim([0, 640])
        ax.set_ylim([0, 480])
        ax.set_zlim([0, 10])
        ax.set_xlabel("X (Left/Right)")
        ax.set_ylabel("Y (Up/Down)")
        ax.set_zlabel("Z (Forward/Backward)")

        plt.draw()
        plt.pause(0.1)  # Small pause to allow the figure to update

        if time.time() - instant_received > .1:
            continue

        ax.clear()  # Clear previous points
        ax.scatter(x_coords, y_coords, z_coords, color=['red', 'orange', 'black', 'green', 'blue', 'purple'])
        print(f"Left Wrist: ({x_coords[2]}, {y_coords[2]}, {z_coords[2]})")

#        plt.draw()
 #       plt.pause(0.1)  # Small pause to allow the figure to update



    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
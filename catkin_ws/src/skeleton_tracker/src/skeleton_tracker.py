#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class SkeletonTracker:
    def __init__(self):
        rospy.init_node('skeleton_tracker')
        
        # init MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # init ROS publishers
        self.pose_pub = rospy.Publisher('skeleton_pose', PoseArray, queue_size=10)
        self.image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)
        
        # init camera
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # BGR to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)
        
        if results.pose_landmarks:
            # PoseArray message
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "camera_frame"
            
            # make ROS poses
            for landmark in results.pose_landmarks.landmark:
                pose = Pose()
                pose.position.x = landmark.x
                pose.position.y = landmark.y
                pose.position.z = landmark.z
                pose_array.poses.append(pose)
                
            self.pose_pub.publish(pose_array)
            
            # pub  image
            mp.solutions.drawing_utils.draw_landmarks(
                frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS
            )
            
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(image_msg)
            
    def run(self):
        rate = rospy.Rate(30)  # 30 Hz had issues with my laptop. Maybe I need an nvidia GPU
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()
            
    def cleanup(self):
        self.cap.release()
        self.pose.close()

if __name__ == '__main__':
    tracker = SkeletonTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        tracker.cleanup()
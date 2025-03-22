import rospy
import heapq
from _0_point_getting import publish_frame as publish_frames
from _1_bodypoints import get_bodypoints
from _2_processing import process_bodypoints
from _3_angle_publishing import publish_angles
from custom_types import *
import threading
from util import timestamp

def app():
    rospy.init_node('POINT_PROCESSOR', anonymous=True)

    frame_queue : q_Frame_Pair_t = []
    bodypoints_queue : q_Bodypoints_t = []
    robotangles_queue : q_Robot_Angles_t = []

    # Initiates a spinning loop that will run indefinitely to fill the queue with camera frames; spawns a thread to do so
    camera_thread = threading.Thread(target=publish_frames, args=(frame_queue))
    camera_thread.start()

    while not rospy.is_shutdown():
        pre_ts = timestamp()
        
        if len(frame_queue) > 0:
            camera_frame = heapq.heappop(frame_queue)
            get_bodypoints(camera_frame, bodypoints_queue)
        else:
            print("  SKIP camera")
        
        if len(bodypoints_queue) > 0:
            bodypoints_frame = heapq.heappop(bodypoints_queue)
            process_bodypoints(bodypoints_frame, robotangles_queue)
        else:
            print("  SKIP bodypoints")
        
        if (len(robotangles_queue)) > 0:
            robotangles_frame = heapq.heappop(robotangles_queue)
            publish_angles(robotangles_frame)
        else:
            print("  SKIP robotangles")
        
        post_ts = timestamp()
        print(f"Completed framework loop in {post_ts - pre_ts}ms!")
    
    
if __name__ == '__main__':
    try:
        app()
    except rospy.ROSInterruptException:
        pass
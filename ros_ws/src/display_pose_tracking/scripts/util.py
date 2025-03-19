import pyrealsense2 as rs

CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480

def setup_depth_pipeline():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("Must have RGB Depth Camera")
        exit(0)

    config.enable_stream(rs.stream.depth, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT, rs.format.rgb8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline

def make_detect_async_callback(depth_frame):
    def callback(result: PoseLandmarkerResult, output_image : mp.Image, timestamp_ms: int):
        if len(result.pose_landmarks) == 0:
            return

        relevant_landmarks = get_relevant_landmarks(result)
        print(f"Arm: {relevant_landmarks}")

        landmarks_with_depth = []
        for point in relevant_landmarks:
            x_coord = round(point.x * 640)
            y_coord = round(point.y * 480)
            if x_coord >= CAMERA_WIDTH or x_coord < 0 or y_coord < 0 or y_coord >= CAMERA_HEIGHT:
                print(f"OUT OF BOUNDS ({x_coord}, {y_coord})")
                landmarks_with_depth.append((x_coord, y_coord, 0))
                return

            z_coord = depth_frame.get_distance(x_coord, y_coord)
            landmarks_with_depth.append((x_coord, y_coord, z_coord))
            print(f"READ ({int(time()*1000) - timestamp_ms}ms): {landmarks_with_depth}")

        msg = landmarks()
        msg.left_hip = landmarks_with_depth[0]
        msg.right_hip = landmarks_with_depth[1]
        msg.left_shoulder = landmarks_with_depth[2]
        msg.right_shoulder = landmarks_with_depth[3]
        msg.left_elbow = landmarks_with_depth[4]
        msg.right_elbow = landmarks_with_depth[5]
        msg.left_wrist = landmarks_with_depth[6]
        msg.right_wrist = landmarks_with_depth[7]
        msg.left_pinky = landmarks_with_depth[8]
        msg.right_pinky = landmarks_with_depth[9]
        msg.left_index = landmarks_with_depth[10]
        msg.right_index = landmarks_with_depth[11]
        msg.left_thumb = landmarks_with_depth[12]
        msg.right_thumb = landmarks_with_depth[13]
        msg.nose = landmarks_with_depth[14]


        # rospy.loginfo(msg) # will continue to log in CLI msg being sent
        print("sent!")
        pub.publish(msg)

    return callback

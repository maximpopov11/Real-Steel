;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::arm)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'arm (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::ARM")
  (make-package "CUSTOM_MSG::ARM"))

(in-package "ROS")
;;//! \htmlinclude arm.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msg::arm
  :super ros::object
  :slots (_header _shoulder _elbow _wrist ))

(defmethod custom_msg::arm
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:shoulder __shoulder) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:elbow __elbow) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:wrist __wrist) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _shoulder __shoulder)
   (setq _elbow __elbow)
   (setq _wrist __wrist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:shoulder
   (&optional __shoulder)
   (if __shoulder (setq _shoulder __shoulder)) _shoulder)
  (:elbow
   (&optional __elbow)
   (if __elbow (setq _elbow __elbow)) _elbow)
  (:wrist
   (&optional __wrist)
   (if __wrist (setq _wrist __wrist)) _wrist)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[3] _shoulder
    (* 4    3)
    ;; float32[3] _elbow
    (* 4    3)
    ;; float32[3] _wrist
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[3] _shoulder
     (dotimes (i 3)
       (sys::poke (elt _shoulder i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _elbow
     (dotimes (i 3)
       (sys::poke (elt _elbow i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _wrist
     (dotimes (i 3)
       (sys::poke (elt _wrist i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[3] _shoulder
   (dotimes (i (length _shoulder))
     (setf (elt _shoulder i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _elbow
   (dotimes (i (length _elbow))
     (setf (elt _elbow i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _wrist
   (dotimes (i (length _wrist))
     (setf (elt _wrist i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get custom_msg::arm :md5sum-) "f17acee9610e62286ac9052ebccb36dc")
(setf (get custom_msg::arm :datatype-) "custom_msg/arm")
(setf (get custom_msg::arm :definition-)
      "Header header

float32[3] shoulder
float32[3] elbow
float32[3] wrist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :custom_msg/arm "f17acee9610e62286ac9052ebccb36dc")



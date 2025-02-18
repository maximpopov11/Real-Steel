; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude arm.msg.html

(cl:defclass <arm> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (shoulder
    :reader shoulder
    :initarg :shoulder
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (elbow
    :reader elbow
    :initarg :elbow
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (wrist
    :reader wrist
    :initarg :wrist
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass arm (<arm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<arm> is deprecated: use custom_msg-msg:arm instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:header-val is deprecated.  Use custom_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'shoulder-val :lambda-list '(m))
(cl:defmethod shoulder-val ((m <arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:shoulder-val is deprecated.  Use custom_msg-msg:shoulder instead.")
  (shoulder m))

(cl:ensure-generic-function 'elbow-val :lambda-list '(m))
(cl:defmethod elbow-val ((m <arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:elbow-val is deprecated.  Use custom_msg-msg:elbow instead.")
  (elbow m))

(cl:ensure-generic-function 'wrist-val :lambda-list '(m))
(cl:defmethod wrist-val ((m <arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:wrist-val is deprecated.  Use custom_msg-msg:wrist instead.")
  (wrist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arm>) ostream)
  "Serializes a message object of type '<arm>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'shoulder))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'elbow))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'wrist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arm>) istream)
  "Deserializes a message object of type '<arm>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'shoulder) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'shoulder)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'elbow) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'elbow)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'wrist) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'wrist)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arm>)))
  "Returns string type for a message object of type '<arm>"
  "custom_msg/arm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arm)))
  "Returns string type for a message object of type 'arm"
  "custom_msg/arm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arm>)))
  "Returns md5sum for a message object of type '<arm>"
  "f17acee9610e62286ac9052ebccb36dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arm)))
  "Returns md5sum for a message object of type 'arm"
  "f17acee9610e62286ac9052ebccb36dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arm>)))
  "Returns full string definition for message of type '<arm>"
  (cl:format cl:nil "Header header~%~%float32[3] shoulder~%float32[3] elbow~%float32[3] wrist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arm)))
  "Returns full string definition for message of type 'arm"
  (cl:format cl:nil "Header header~%~%float32[3] shoulder~%float32[3] elbow~%float32[3] wrist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arm>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'shoulder) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'elbow) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'wrist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arm>))
  "Converts a ROS message object to a list"
  (cl:list 'arm
    (cl:cons ':header (header msg))
    (cl:cons ':shoulder (shoulder msg))
    (cl:cons ':elbow (elbow msg))
    (cl:cons ':wrist (wrist msg))
))

; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude railLineVector.msg.html

(cl:defclass <railLineVector> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lines
    :reader lines
    :initarg :lines
    :type (cl:vector acri_localisation-msg:railLine)
   :initform (cl:make-array 0 :element-type 'acri_localisation-msg:railLine :initial-element (cl:make-instance 'acri_localisation-msg:railLine))))
)

(cl:defclass railLineVector (<railLineVector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <railLineVector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'railLineVector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<railLineVector> is deprecated: use acri_localisation-msg:railLineVector instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <railLineVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:header-val is deprecated.  Use acri_localisation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lines-val :lambda-list '(m))
(cl:defmethod lines-val ((m <railLineVector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:lines-val is deprecated.  Use acri_localisation-msg:lines instead.")
  (lines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <railLineVector>) ostream)
  "Serializes a message object of type '<railLineVector>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <railLineVector>) istream)
  "Deserializes a message object of type '<railLineVector>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'acri_localisation-msg:railLine))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<railLineVector>)))
  "Returns string type for a message object of type '<railLineVector>"
  "acri_localisation/railLineVector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'railLineVector)))
  "Returns string type for a message object of type 'railLineVector"
  "acri_localisation/railLineVector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<railLineVector>)))
  "Returns md5sum for a message object of type '<railLineVector>"
  "61e2ffd9765f40d10071b4476bd6929a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'railLineVector)))
  "Returns md5sum for a message object of type 'railLineVector"
  "61e2ffd9765f40d10071b4476bd6929a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<railLineVector>)))
  "Returns full string definition for message of type '<railLineVector>"
  (cl:format cl:nil "std_msgs/Header header~%acri_localisation/railLine[] lines~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'railLineVector)))
  "Returns full string definition for message of type 'railLineVector"
  (cl:format cl:nil "std_msgs/Header header~%acri_localisation/railLine[] lines~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <railLineVector>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <railLineVector>))
  "Converts a ROS message object to a list"
  (cl:list 'railLineVector
    (cl:cons ':header (header msg))
    (cl:cons ':lines (lines msg))
))

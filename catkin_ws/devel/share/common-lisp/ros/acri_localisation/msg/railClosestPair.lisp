; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude railClosestPair.msg.html

(cl:defclass <railClosestPair> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (line1
    :reader line1
    :initarg :line1
    :type acri_localisation-msg:railLine
    :initform (cl:make-instance 'acri_localisation-msg:railLine))
   (line2
    :reader line2
    :initarg :line2
    :type acri_localisation-msg:railLine
    :initform (cl:make-instance 'acri_localisation-msg:railLine))
   (midline
    :reader midline
    :initarg :midline
    :type acri_localisation-msg:railLine
    :initform (cl:make-instance 'acri_localisation-msg:railLine))
   (inrange
    :reader inrange
    :initarg :inrange
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass railClosestPair (<railClosestPair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <railClosestPair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'railClosestPair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<railClosestPair> is deprecated: use acri_localisation-msg:railClosestPair instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <railClosestPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:header-val is deprecated.  Use acri_localisation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'line1-val :lambda-list '(m))
(cl:defmethod line1-val ((m <railClosestPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:line1-val is deprecated.  Use acri_localisation-msg:line1 instead.")
  (line1 m))

(cl:ensure-generic-function 'line2-val :lambda-list '(m))
(cl:defmethod line2-val ((m <railClosestPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:line2-val is deprecated.  Use acri_localisation-msg:line2 instead.")
  (line2 m))

(cl:ensure-generic-function 'midline-val :lambda-list '(m))
(cl:defmethod midline-val ((m <railClosestPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:midline-val is deprecated.  Use acri_localisation-msg:midline instead.")
  (midline m))

(cl:ensure-generic-function 'inrange-val :lambda-list '(m))
(cl:defmethod inrange-val ((m <railClosestPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:inrange-val is deprecated.  Use acri_localisation-msg:inrange instead.")
  (inrange m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <railClosestPair>) ostream)
  "Serializes a message object of type '<railClosestPair>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'line1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'line2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'midline) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'inrange) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <railClosestPair>) istream)
  "Deserializes a message object of type '<railClosestPair>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'line1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'line2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'midline) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'inrange) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<railClosestPair>)))
  "Returns string type for a message object of type '<railClosestPair>"
  "acri_localisation/railClosestPair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'railClosestPair)))
  "Returns string type for a message object of type 'railClosestPair"
  "acri_localisation/railClosestPair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<railClosestPair>)))
  "Returns md5sum for a message object of type '<railClosestPair>"
  "1f8485efdc65afe9c5ad5817e1a262cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'railClosestPair)))
  "Returns md5sum for a message object of type 'railClosestPair"
  "1f8485efdc65afe9c5ad5817e1a262cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<railClosestPair>)))
  "Returns full string definition for message of type '<railClosestPair>"
  (cl:format cl:nil "std_msgs/Header header~%acri_localisation/railLine line1~%acri_localisation/railLine line2~%acri_localisation/railLine midline~%std_msgs/Bool inrange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'railClosestPair)))
  "Returns full string definition for message of type 'railClosestPair"
  (cl:format cl:nil "std_msgs/Header header~%acri_localisation/railLine line1~%acri_localisation/railLine line2~%acri_localisation/railLine midline~%std_msgs/Bool inrange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <railClosestPair>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'line1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'line2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'midline))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'inrange))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <railClosestPair>))
  "Converts a ROS message object to a list"
  (cl:list 'railClosestPair
    (cl:cons ':header (header msg))
    (cl:cons ':line1 (line1 msg))
    (cl:cons ':line2 (line2 msg))
    (cl:cons ':midline (midline msg))
    (cl:cons ':inrange (inrange msg))
))

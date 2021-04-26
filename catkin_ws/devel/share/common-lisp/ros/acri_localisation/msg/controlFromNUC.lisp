; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude controlFromNUC.msg.html

(cl:defclass <controlFromNUC> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (midline
    :reader midline
    :initarg :midline
    :type acri_localisation-msg:railLine
    :initform (cl:make-instance 'acri_localisation-msg:railLine))
   (pose2D
    :reader pose2D
    :initarg :pose2D
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (mode
    :reader mode
    :initarg :mode
    :type std_msgs-msg:UInt32
    :initform (cl:make-instance 'std_msgs-msg:UInt32))
   (inrange
    :reader inrange
    :initarg :inrange
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (voltage24
    :reader voltage24
    :initarg :voltage24
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (voltage48
    :reader voltage48
    :initarg :voltage48
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass controlFromNUC (<controlFromNUC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controlFromNUC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controlFromNUC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<controlFromNUC> is deprecated: use acri_localisation-msg:controlFromNUC instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:header-val is deprecated.  Use acri_localisation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'midline-val :lambda-list '(m))
(cl:defmethod midline-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:midline-val is deprecated.  Use acri_localisation-msg:midline instead.")
  (midline m))

(cl:ensure-generic-function 'pose2D-val :lambda-list '(m))
(cl:defmethod pose2D-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:pose2D-val is deprecated.  Use acri_localisation-msg:pose2D instead.")
  (pose2D m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:mode-val is deprecated.  Use acri_localisation-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'inrange-val :lambda-list '(m))
(cl:defmethod inrange-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:inrange-val is deprecated.  Use acri_localisation-msg:inrange instead.")
  (inrange m))

(cl:ensure-generic-function 'voltage24-val :lambda-list '(m))
(cl:defmethod voltage24-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:voltage24-val is deprecated.  Use acri_localisation-msg:voltage24 instead.")
  (voltage24 m))

(cl:ensure-generic-function 'voltage48-val :lambda-list '(m))
(cl:defmethod voltage48-val ((m <controlFromNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:voltage48-val is deprecated.  Use acri_localisation-msg:voltage48 instead.")
  (voltage48 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controlFromNUC>) ostream)
  "Serializes a message object of type '<controlFromNUC>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'midline) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose2D) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'inrange) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'voltage24) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'voltage48) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controlFromNUC>) istream)
  "Deserializes a message object of type '<controlFromNUC>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'midline) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose2D) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'inrange) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'voltage24) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'voltage48) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controlFromNUC>)))
  "Returns string type for a message object of type '<controlFromNUC>"
  "acri_localisation/controlFromNUC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controlFromNUC)))
  "Returns string type for a message object of type 'controlFromNUC"
  "acri_localisation/controlFromNUC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controlFromNUC>)))
  "Returns md5sum for a message object of type '<controlFromNUC>"
  "d79800d54a13d23f4d7ba0216eb0f324")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controlFromNUC)))
  "Returns md5sum for a message object of type 'controlFromNUC"
  "d79800d54a13d23f4d7ba0216eb0f324")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controlFromNUC>)))
  "Returns full string definition for message of type '<controlFromNUC>"
  (cl:format cl:nil "# header: sequence and time-step id~%# mid line: rail mid_line comprising of two 3D points~%# pose2D: robot pose comprising of [x,y,theta] ~%# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault~%# inrange: rail is within valid range of vehicle to go into autonomous mode~%# voltage24: voltage of 24V battery~%# voltage48: voltage of 48V battery~%~%std_msgs/Header header~%acri_localisation/railLine midline~%geometry_msgs/Pose2D pose2D~%std_msgs/UInt32 mode~%std_msgs/Bool inrange~%std_msgs/Float32 voltage24~%std_msgs/Float32 voltage48~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: std_msgs/UInt32~%uint32 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controlFromNUC)))
  "Returns full string definition for message of type 'controlFromNUC"
  (cl:format cl:nil "# header: sequence and time-step id~%# mid line: rail mid_line comprising of two 3D points~%# pose2D: robot pose comprising of [x,y,theta] ~%# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault~%# inrange: rail is within valid range of vehicle to go into autonomous mode~%# voltage24: voltage of 24V battery~%# voltage48: voltage of 48V battery~%~%std_msgs/Header header~%acri_localisation/railLine midline~%geometry_msgs/Pose2D pose2D~%std_msgs/UInt32 mode~%std_msgs/Bool inrange~%std_msgs/Float32 voltage24~%std_msgs/Float32 voltage48~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: std_msgs/UInt32~%uint32 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controlFromNUC>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'midline))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose2D))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'inrange))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'voltage24))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'voltage48))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controlFromNUC>))
  "Converts a ROS message object to a list"
  (cl:list 'controlFromNUC
    (cl:cons ':header (header msg))
    (cl:cons ':midline (midline msg))
    (cl:cons ':pose2D (pose2D msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':inrange (inrange msg))
    (cl:cons ':voltage24 (voltage24 msg))
    (cl:cons ':voltage48 (voltage48 msg))
))

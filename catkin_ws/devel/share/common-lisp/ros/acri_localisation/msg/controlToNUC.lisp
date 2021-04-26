; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude controlToNUC.msg.html

(cl:defclass <controlToNUC> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type std_msgs-msg:UInt32
    :initform (cl:make-instance 'std_msgs-msg:UInt32))
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

(cl:defclass controlToNUC (<controlToNUC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controlToNUC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controlToNUC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<controlToNUC> is deprecated: use acri_localisation-msg:controlToNUC instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <controlToNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:header-val is deprecated.  Use acri_localisation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <controlToNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:mode-val is deprecated.  Use acri_localisation-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'voltage24-val :lambda-list '(m))
(cl:defmethod voltage24-val ((m <controlToNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:voltage24-val is deprecated.  Use acri_localisation-msg:voltage24 instead.")
  (voltage24 m))

(cl:ensure-generic-function 'voltage48-val :lambda-list '(m))
(cl:defmethod voltage48-val ((m <controlToNUC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:voltage48-val is deprecated.  Use acri_localisation-msg:voltage48 instead.")
  (voltage48 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controlToNUC>) ostream)
  "Serializes a message object of type '<controlToNUC>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'voltage24) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'voltage48) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controlToNUC>) istream)
  "Deserializes a message object of type '<controlToNUC>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'voltage24) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'voltage48) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controlToNUC>)))
  "Returns string type for a message object of type '<controlToNUC>"
  "acri_localisation/controlToNUC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controlToNUC)))
  "Returns string type for a message object of type 'controlToNUC"
  "acri_localisation/controlToNUC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controlToNUC>)))
  "Returns md5sum for a message object of type '<controlToNUC>"
  "3fae9323d5973f4783a5e2f3a2ec3199")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controlToNUC)))
  "Returns md5sum for a message object of type 'controlToNUC"
  "3fae9323d5973f4783a5e2f3a2ec3199")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controlToNUC>)))
  "Returns full string definition for message of type '<controlToNUC>"
  (cl:format cl:nil "# header: sequence and time-step id~%# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault~%# voltage24: voltage of 24V battery~%# voltage48: voltage of 48V battery~%std_msgs/Header header~%std_msgs/UInt32 mode~%std_msgs/Float32 voltage24~%std_msgs/Float32 voltage48~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/UInt32~%uint32 data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controlToNUC)))
  "Returns full string definition for message of type 'controlToNUC"
  (cl:format cl:nil "# header: sequence and time-step id~%# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault~%# voltage24: voltage of 24V battery~%# voltage48: voltage of 48V battery~%std_msgs/Header header~%std_msgs/UInt32 mode~%std_msgs/Float32 voltage24~%std_msgs/Float32 voltage48~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/UInt32~%uint32 data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controlToNUC>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'voltage24))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'voltage48))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controlToNUC>))
  "Converts a ROS message object to a list"
  (cl:list 'controlToNUC
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':voltage24 (voltage24 msg))
    (cl:cons ':voltage48 (voltage48 msg))
))

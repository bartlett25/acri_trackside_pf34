; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude railPair.msg.html

(cl:defclass <railPair> (roslisp-msg-protocol:ros-message)
  ((line1
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
    :initform (cl:make-instance 'acri_localisation-msg:railLine)))
)

(cl:defclass railPair (<railPair>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <railPair>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'railPair)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<railPair> is deprecated: use acri_localisation-msg:railPair instead.")))

(cl:ensure-generic-function 'line1-val :lambda-list '(m))
(cl:defmethod line1-val ((m <railPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:line1-val is deprecated.  Use acri_localisation-msg:line1 instead.")
  (line1 m))

(cl:ensure-generic-function 'line2-val :lambda-list '(m))
(cl:defmethod line2-val ((m <railPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:line2-val is deprecated.  Use acri_localisation-msg:line2 instead.")
  (line2 m))

(cl:ensure-generic-function 'midline-val :lambda-list '(m))
(cl:defmethod midline-val ((m <railPair>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:midline-val is deprecated.  Use acri_localisation-msg:midline instead.")
  (midline m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <railPair>) ostream)
  "Serializes a message object of type '<railPair>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'line1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'line2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'midline) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <railPair>) istream)
  "Deserializes a message object of type '<railPair>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'line1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'line2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'midline) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<railPair>)))
  "Returns string type for a message object of type '<railPair>"
  "acri_localisation/railPair")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'railPair)))
  "Returns string type for a message object of type 'railPair"
  "acri_localisation/railPair")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<railPair>)))
  "Returns md5sum for a message object of type '<railPair>"
  "d47e372c1831bf7059fde9b5559b9f11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'railPair)))
  "Returns md5sum for a message object of type 'railPair"
  "d47e372c1831bf7059fde9b5559b9f11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<railPair>)))
  "Returns full string definition for message of type '<railPair>"
  (cl:format cl:nil "acri_localisation/railLine line1~%acri_localisation/railLine line2~%acri_localisation/railLine midline~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'railPair)))
  "Returns full string definition for message of type 'railPair"
  (cl:format cl:nil "acri_localisation/railLine line1~%acri_localisation/railLine line2~%acri_localisation/railLine midline~%================================================================================~%MSG: acri_localisation/railLine~%geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <railPair>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'line1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'line2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'midline))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <railPair>))
  "Converts a ROS message object to a list"
  (cl:list 'railPair
    (cl:cons ':line1 (line1 msg))
    (cl:cons ':line2 (line2 msg))
    (cl:cons ':midline (midline msg))
))

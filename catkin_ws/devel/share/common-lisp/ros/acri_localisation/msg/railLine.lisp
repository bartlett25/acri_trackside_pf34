; Auto-generated. Do not edit!


(cl:in-package acri_localisation-msg)


;//! \htmlinclude railLine.msg.html

(cl:defclass <railLine> (roslisp-msg-protocol:ros-message)
  ((point1
    :reader point1
    :initarg :point1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (point2
    :reader point2
    :initarg :point2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass railLine (<railLine>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <railLine>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'railLine)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acri_localisation-msg:<railLine> is deprecated: use acri_localisation-msg:railLine instead.")))

(cl:ensure-generic-function 'point1-val :lambda-list '(m))
(cl:defmethod point1-val ((m <railLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:point1-val is deprecated.  Use acri_localisation-msg:point1 instead.")
  (point1 m))

(cl:ensure-generic-function 'point2-val :lambda-list '(m))
(cl:defmethod point2-val ((m <railLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acri_localisation-msg:point2-val is deprecated.  Use acri_localisation-msg:point2 instead.")
  (point2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <railLine>) ostream)
  "Serializes a message object of type '<railLine>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <railLine>) istream)
  "Deserializes a message object of type '<railLine>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<railLine>)))
  "Returns string type for a message object of type '<railLine>"
  "acri_localisation/railLine")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'railLine)))
  "Returns string type for a message object of type 'railLine"
  "acri_localisation/railLine")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<railLine>)))
  "Returns md5sum for a message object of type '<railLine>"
  "e377648347b19d625c7a86b684f82b75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'railLine)))
  "Returns md5sum for a message object of type 'railLine"
  "e377648347b19d625c7a86b684f82b75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<railLine>)))
  "Returns full string definition for message of type '<railLine>"
  (cl:format cl:nil "geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'railLine)))
  "Returns full string definition for message of type 'railLine"
  (cl:format cl:nil "geometry_msgs/Point point1~%geometry_msgs/Point point2~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <railLine>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <railLine>))
  "Converts a ROS message object to a list"
  (cl:list 'railLine
    (cl:cons ':point1 (point1 msg))
    (cl:cons ':point2 (point2 msg))
))

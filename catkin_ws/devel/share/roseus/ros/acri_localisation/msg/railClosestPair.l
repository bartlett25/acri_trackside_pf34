;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::railClosestPair)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'railClosestPair (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::RAILCLOSESTPAIR")
  (make-package "ACRI_LOCALISATION::RAILCLOSESTPAIR"))

(in-package "ROS")
;;//! \htmlinclude railClosestPair.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass acri_localisation::railClosestPair
  :super ros::object
  :slots (_header _line1 _line2 _midline _inrange ))

(defmethod acri_localisation::railClosestPair
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:line1 __line1) (instance acri_localisation::railLine :init))
    ((:line2 __line2) (instance acri_localisation::railLine :init))
    ((:midline __midline) (instance acri_localisation::railLine :init))
    ((:inrange __inrange) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _line1 __line1)
   (setq _line2 __line2)
   (setq _midline __midline)
   (setq _inrange __inrange)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:line1
   (&rest __line1)
   (if (keywordp (car __line1))
       (send* _line1 __line1)
     (progn
       (if __line1 (setq _line1 (car __line1)))
       _line1)))
  (:line2
   (&rest __line2)
   (if (keywordp (car __line2))
       (send* _line2 __line2)
     (progn
       (if __line2 (setq _line2 (car __line2)))
       _line2)))
  (:midline
   (&rest __midline)
   (if (keywordp (car __midline))
       (send* _midline __midline)
     (progn
       (if __midline (setq _midline (car __midline)))
       _midline)))
  (:inrange
   (&rest __inrange)
   (if (keywordp (car __inrange))
       (send* _inrange __inrange)
     (progn
       (if __inrange (setq _inrange (car __inrange)))
       _inrange)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; acri_localisation/railLine _line1
    (send _line1 :serialization-length)
    ;; acri_localisation/railLine _line2
    (send _line2 :serialization-length)
    ;; acri_localisation/railLine _midline
    (send _midline :serialization-length)
    ;; std_msgs/Bool _inrange
    (send _inrange :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; acri_localisation/railLine _line1
       (send _line1 :serialize s)
     ;; acri_localisation/railLine _line2
       (send _line2 :serialize s)
     ;; acri_localisation/railLine _midline
       (send _midline :serialize s)
     ;; std_msgs/Bool _inrange
       (send _inrange :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; acri_localisation/railLine _line1
     (send _line1 :deserialize buf ptr-) (incf ptr- (send _line1 :serialization-length))
   ;; acri_localisation/railLine _line2
     (send _line2 :deserialize buf ptr-) (incf ptr- (send _line2 :serialization-length))
   ;; acri_localisation/railLine _midline
     (send _midline :deserialize buf ptr-) (incf ptr- (send _midline :serialization-length))
   ;; std_msgs/Bool _inrange
     (send _inrange :deserialize buf ptr-) (incf ptr- (send _inrange :serialization-length))
   ;;
   self)
  )

(setf (get acri_localisation::railClosestPair :md5sum-) "1f8485efdc65afe9c5ad5817e1a262cf")
(setf (get acri_localisation::railClosestPair :datatype-) "acri_localisation/railClosestPair")
(setf (get acri_localisation::railClosestPair :definition-)
      "std_msgs/Header header
acri_localisation/railLine line1
acri_localisation/railLine line2
acri_localisation/railLine midline
std_msgs/Bool inrange
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

================================================================================
MSG: acri_localisation/railLine
geometry_msgs/Point point1
geometry_msgs/Point point2
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :acri_localisation/railClosestPair "1f8485efdc65afe9c5ad5817e1a262cf")



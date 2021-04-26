;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::railPairVector)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'railPairVector (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::RAILPAIRVECTOR")
  (make-package "ACRI_LOCALISATION::RAILPAIRVECTOR"))

(in-package "ROS")
;;//! \htmlinclude railPairVector.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass acri_localisation::railPairVector
  :super ros::object
  :slots (_header _pairs ))

(defmethod acri_localisation::railPairVector
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pairs __pairs) (let (r) (dotimes (i 0) (push (instance acri_localisation::railPair :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pairs __pairs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pairs
   (&rest __pairs)
   (if (keywordp (car __pairs))
       (send* _pairs __pairs)
     (progn
       (if __pairs (setq _pairs (car __pairs)))
       _pairs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; acri_localisation/railPair[] _pairs
    (apply #'+ (send-all _pairs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; acri_localisation/railPair[] _pairs
     (write-long (length _pairs) s)
     (dolist (elem _pairs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; acri_localisation/railPair[] _pairs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pairs (let (r) (dotimes (i n) (push (instance acri_localisation::railPair :init) r)) r))
     (dolist (elem- _pairs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get acri_localisation::railPairVector :md5sum-) "45d6f0b4412639e96eb4e308a8b6852c")
(setf (get acri_localisation::railPairVector :datatype-) "acri_localisation/railPairVector")
(setf (get acri_localisation::railPairVector :definition-)
      "std_msgs/Header header
acri_localisation/railPair[] pairs
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
MSG: acri_localisation/railPair
acri_localisation/railLine line1
acri_localisation/railLine line2
acri_localisation/railLine midline
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

")



(provide :acri_localisation/railPairVector "45d6f0b4412639e96eb4e308a8b6852c")



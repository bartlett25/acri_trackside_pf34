;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::railLineVector)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'railLineVector (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::RAILLINEVECTOR")
  (make-package "ACRI_LOCALISATION::RAILLINEVECTOR"))

(in-package "ROS")
;;//! \htmlinclude railLineVector.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass acri_localisation::railLineVector
  :super ros::object
  :slots (_header _lines ))

(defmethod acri_localisation::railLineVector
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance acri_localisation::railLine :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lines __lines)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; acri_localisation/railLine[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; acri_localisation/railLine[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; acri_localisation/railLine[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance acri_localisation::railLine :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get acri_localisation::railLineVector :md5sum-) "61e2ffd9765f40d10071b4476bd6929a")
(setf (get acri_localisation::railLineVector :datatype-) "acri_localisation/railLineVector")
(setf (get acri_localisation::railLineVector :definition-)
      "std_msgs/Header header
acri_localisation/railLine[] lines
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

")



(provide :acri_localisation/railLineVector "61e2ffd9765f40d10071b4476bd6929a")



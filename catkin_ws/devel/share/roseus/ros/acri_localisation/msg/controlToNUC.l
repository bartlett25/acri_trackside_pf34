;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::controlToNUC)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'controlToNUC (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::CONTROLTONUC")
  (make-package "ACRI_LOCALISATION::CONTROLTONUC"))

(in-package "ROS")
;;//! \htmlinclude controlToNUC.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass acri_localisation::controlToNUC
  :super ros::object
  :slots (_header _mode _voltage24 _voltage48 ))

(defmethod acri_localisation::controlToNUC
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) (instance std_msgs::UInt32 :init))
    ((:voltage24 __voltage24) (instance std_msgs::Float32 :init))
    ((:voltage48 __voltage48) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode __mode)
   (setq _voltage24 __voltage24)
   (setq _voltage48 __voltage48)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&rest __mode)
   (if (keywordp (car __mode))
       (send* _mode __mode)
     (progn
       (if __mode (setq _mode (car __mode)))
       _mode)))
  (:voltage24
   (&rest __voltage24)
   (if (keywordp (car __voltage24))
       (send* _voltage24 __voltage24)
     (progn
       (if __voltage24 (setq _voltage24 (car __voltage24)))
       _voltage24)))
  (:voltage48
   (&rest __voltage48)
   (if (keywordp (car __voltage48))
       (send* _voltage48 __voltage48)
     (progn
       (if __voltage48 (setq _voltage48 (car __voltage48)))
       _voltage48)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/UInt32 _mode
    (send _mode :serialization-length)
    ;; std_msgs/Float32 _voltage24
    (send _voltage24 :serialization-length)
    ;; std_msgs/Float32 _voltage48
    (send _voltage48 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/UInt32 _mode
       (send _mode :serialize s)
     ;; std_msgs/Float32 _voltage24
       (send _voltage24 :serialize s)
     ;; std_msgs/Float32 _voltage48
       (send _voltage48 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/UInt32 _mode
     (send _mode :deserialize buf ptr-) (incf ptr- (send _mode :serialization-length))
   ;; std_msgs/Float32 _voltage24
     (send _voltage24 :deserialize buf ptr-) (incf ptr- (send _voltage24 :serialization-length))
   ;; std_msgs/Float32 _voltage48
     (send _voltage48 :deserialize buf ptr-) (incf ptr- (send _voltage48 :serialization-length))
   ;;
   self)
  )

(setf (get acri_localisation::controlToNUC :md5sum-) "3fae9323d5973f4783a5e2f3a2ec3199")
(setf (get acri_localisation::controlToNUC :datatype-) "acri_localisation/controlToNUC")
(setf (get acri_localisation::controlToNUC :definition-)
      "# header: sequence and time-step id
# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
# voltage24: voltage of 24V battery
# voltage48: voltage of 48V battery
std_msgs/Header header
std_msgs/UInt32 mode
std_msgs/Float32 voltage24
std_msgs/Float32 voltage48
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
MSG: std_msgs/UInt32
uint32 data
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :acri_localisation/controlToNUC "3fae9323d5973f4783a5e2f3a2ec3199")



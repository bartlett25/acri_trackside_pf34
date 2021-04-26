;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::controlFromNUC)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'controlFromNUC (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::CONTROLFROMNUC")
  (make-package "ACRI_LOCALISATION::CONTROLFROMNUC"))

(in-package "ROS")
;;//! \htmlinclude controlFromNUC.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass acri_localisation::controlFromNUC
  :super ros::object
  :slots (_header _midline _pose2D _mode _inrange _voltage24 _voltage48 ))

(defmethod acri_localisation::controlFromNUC
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:midline __midline) (instance acri_localisation::railLine :init))
    ((:pose2D __pose2D) (instance geometry_msgs::Pose2D :init))
    ((:mode __mode) (instance std_msgs::UInt32 :init))
    ((:inrange __inrange) (instance std_msgs::Bool :init))
    ((:voltage24 __voltage24) (instance std_msgs::Float32 :init))
    ((:voltage48 __voltage48) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _midline __midline)
   (setq _pose2D __pose2D)
   (setq _mode __mode)
   (setq _inrange __inrange)
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
  (:midline
   (&rest __midline)
   (if (keywordp (car __midline))
       (send* _midline __midline)
     (progn
       (if __midline (setq _midline (car __midline)))
       _midline)))
  (:pose2D
   (&rest __pose2D)
   (if (keywordp (car __pose2D))
       (send* _pose2D __pose2D)
     (progn
       (if __pose2D (setq _pose2D (car __pose2D)))
       _pose2D)))
  (:mode
   (&rest __mode)
   (if (keywordp (car __mode))
       (send* _mode __mode)
     (progn
       (if __mode (setq _mode (car __mode)))
       _mode)))
  (:inrange
   (&rest __inrange)
   (if (keywordp (car __inrange))
       (send* _inrange __inrange)
     (progn
       (if __inrange (setq _inrange (car __inrange)))
       _inrange)))
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
    ;; acri_localisation/railLine _midline
    (send _midline :serialization-length)
    ;; geometry_msgs/Pose2D _pose2D
    (send _pose2D :serialization-length)
    ;; std_msgs/UInt32 _mode
    (send _mode :serialization-length)
    ;; std_msgs/Bool _inrange
    (send _inrange :serialization-length)
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
     ;; acri_localisation/railLine _midline
       (send _midline :serialize s)
     ;; geometry_msgs/Pose2D _pose2D
       (send _pose2D :serialize s)
     ;; std_msgs/UInt32 _mode
       (send _mode :serialize s)
     ;; std_msgs/Bool _inrange
       (send _inrange :serialize s)
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
   ;; acri_localisation/railLine _midline
     (send _midline :deserialize buf ptr-) (incf ptr- (send _midline :serialization-length))
   ;; geometry_msgs/Pose2D _pose2D
     (send _pose2D :deserialize buf ptr-) (incf ptr- (send _pose2D :serialization-length))
   ;; std_msgs/UInt32 _mode
     (send _mode :deserialize buf ptr-) (incf ptr- (send _mode :serialization-length))
   ;; std_msgs/Bool _inrange
     (send _inrange :deserialize buf ptr-) (incf ptr- (send _inrange :serialization-length))
   ;; std_msgs/Float32 _voltage24
     (send _voltage24 :deserialize buf ptr-) (incf ptr- (send _voltage24 :serialization-length))
   ;; std_msgs/Float32 _voltage48
     (send _voltage48 :deserialize buf ptr-) (incf ptr- (send _voltage48 :serialization-length))
   ;;
   self)
  )

(setf (get acri_localisation::controlFromNUC :md5sum-) "d79800d54a13d23f4d7ba0216eb0f324")
(setf (get acri_localisation::controlFromNUC :datatype-) "acri_localisation/controlFromNUC")
(setf (get acri_localisation::controlFromNUC :definition-)
      "# header: sequence and time-step id
# mid line: rail mid_line comprising of two 3D points
# pose2D: robot pose comprising of [x,y,theta] 
# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
# inrange: rail is within valid range of vehicle to go into autonomous mode
# voltage24: voltage of 24V battery
# voltage48: voltage of 48V battery

std_msgs/Header header
acri_localisation/railLine midline
geometry_msgs/Pose2D pose2D
std_msgs/UInt32 mode
std_msgs/Bool inrange
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
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: std_msgs/UInt32
uint32 data
================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :acri_localisation/controlFromNUC "d79800d54a13d23f4d7ba0216eb0f324")



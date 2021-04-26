;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::railLine)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'railLine (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::RAILLINE")
  (make-package "ACRI_LOCALISATION::RAILLINE"))

(in-package "ROS")
;;//! \htmlinclude railLine.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass acri_localisation::railLine
  :super ros::object
  :slots (_point1 _point2 ))

(defmethod acri_localisation::railLine
  (:init
   (&key
    ((:point1 __point1) (instance geometry_msgs::Point :init))
    ((:point2 __point2) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _point1 __point1)
   (setq _point2 __point2)
   self)
  (:point1
   (&rest __point1)
   (if (keywordp (car __point1))
       (send* _point1 __point1)
     (progn
       (if __point1 (setq _point1 (car __point1)))
       _point1)))
  (:point2
   (&rest __point2)
   (if (keywordp (car __point2))
       (send* _point2 __point2)
     (progn
       (if __point2 (setq _point2 (car __point2)))
       _point2)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point1
    (send _point1 :serialization-length)
    ;; geometry_msgs/Point _point2
    (send _point2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point1
       (send _point1 :serialize s)
     ;; geometry_msgs/Point _point2
       (send _point2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point1
     (send _point1 :deserialize buf ptr-) (incf ptr- (send _point1 :serialization-length))
   ;; geometry_msgs/Point _point2
     (send _point2 :deserialize buf ptr-) (incf ptr- (send _point2 :serialization-length))
   ;;
   self)
  )

(setf (get acri_localisation::railLine :md5sum-) "e377648347b19d625c7a86b684f82b75")
(setf (get acri_localisation::railLine :datatype-) "acri_localisation/railLine")
(setf (get acri_localisation::railLine :definition-)
      "geometry_msgs/Point point1
geometry_msgs/Point point2
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :acri_localisation/railLine "e377648347b19d625c7a86b684f82b75")



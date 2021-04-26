;; Auto-generated. Do not edit!


(when (boundp 'acri_localisation::railPair)
  (if (not (find-package "ACRI_LOCALISATION"))
    (make-package "ACRI_LOCALISATION"))
  (shadow 'railPair (find-package "ACRI_LOCALISATION")))
(unless (find-package "ACRI_LOCALISATION::RAILPAIR")
  (make-package "ACRI_LOCALISATION::RAILPAIR"))

(in-package "ROS")
;;//! \htmlinclude railPair.msg.html


(defclass acri_localisation::railPair
  :super ros::object
  :slots (_line1 _line2 _midline ))

(defmethod acri_localisation::railPair
  (:init
   (&key
    ((:line1 __line1) (instance acri_localisation::railLine :init))
    ((:line2 __line2) (instance acri_localisation::railLine :init))
    ((:midline __midline) (instance acri_localisation::railLine :init))
    )
   (send-super :init)
   (setq _line1 __line1)
   (setq _line2 __line2)
   (setq _midline __midline)
   self)
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
  (:serialization-length
   ()
   (+
    ;; acri_localisation/railLine _line1
    (send _line1 :serialization-length)
    ;; acri_localisation/railLine _line2
    (send _line2 :serialization-length)
    ;; acri_localisation/railLine _midline
    (send _midline :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; acri_localisation/railLine _line1
       (send _line1 :serialize s)
     ;; acri_localisation/railLine _line2
       (send _line2 :serialize s)
     ;; acri_localisation/railLine _midline
       (send _midline :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; acri_localisation/railLine _line1
     (send _line1 :deserialize buf ptr-) (incf ptr- (send _line1 :serialization-length))
   ;; acri_localisation/railLine _line2
     (send _line2 :deserialize buf ptr-) (incf ptr- (send _line2 :serialization-length))
   ;; acri_localisation/railLine _midline
     (send _midline :deserialize buf ptr-) (incf ptr- (send _midline :serialization-length))
   ;;
   self)
  )

(setf (get acri_localisation::railPair :md5sum-) "d47e372c1831bf7059fde9b5559b9f11")
(setf (get acri_localisation::railPair :datatype-) "acri_localisation/railPair")
(setf (get acri_localisation::railPair :definition-)
      "acri_localisation/railLine line1
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



(provide :acri_localisation/railPair "d47e372c1831bf7059fde9b5559b9f11")




(cl:in-package :asdf)

(defsystem "acri_localisation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "controlFromNUC" :depends-on ("_package_controlFromNUC"))
    (:file "_package_controlFromNUC" :depends-on ("_package"))
    (:file "controlToNUC" :depends-on ("_package_controlToNUC"))
    (:file "_package_controlToNUC" :depends-on ("_package"))
    (:file "poseEuler" :depends-on ("_package_poseEuler"))
    (:file "_package_poseEuler" :depends-on ("_package"))
    (:file "railClosestPair" :depends-on ("_package_railClosestPair"))
    (:file "_package_railClosestPair" :depends-on ("_package"))
    (:file "railLine" :depends-on ("_package_railLine"))
    (:file "_package_railLine" :depends-on ("_package"))
    (:file "railLineVector" :depends-on ("_package_railLineVector"))
    (:file "_package_railLineVector" :depends-on ("_package"))
    (:file "railPair" :depends-on ("_package_railPair"))
    (:file "_package_railPair" :depends-on ("_package"))
    (:file "railPairVector" :depends-on ("_package_railPairVector"))
    (:file "_package_railPairVector" :depends-on ("_package"))
  ))

(cl:in-package :asdf)

(defsystem "magmed_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MagCR" :depends-on ("_package_MagCR"))
    (:file "_package_MagCR" :depends-on ("_package"))
    (:file "PFjoystick" :depends-on ("_package_PFjoystick"))
    (:file "_package_PFjoystick" :depends-on ("_package"))
    (:file "PoseTwist" :depends-on ("_package_PoseTwist"))
    (:file "_package_PoseTwist" :depends-on ("_package"))
    (:file "RoboJoints" :depends-on ("_package_RoboJoints"))
    (:file "_package_RoboJoints" :depends-on ("_package"))
    (:file "RoboStates" :depends-on ("_package_RoboStates"))
    (:file "_package_RoboStates" :depends-on ("_package"))
    (:file "TipAngle" :depends-on ("_package_TipAngle"))
    (:file "_package_TipAngle" :depends-on ("_package"))
  ))
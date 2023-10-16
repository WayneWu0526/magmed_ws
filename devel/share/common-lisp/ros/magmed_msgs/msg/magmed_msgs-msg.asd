
(cl:in-package :asdf)

(defsystem "magmed_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointsVel" :depends-on ("_package_JointsVel"))
    (:file "_package_JointsVel" :depends-on ("_package"))
    (:file "JoyRef" :depends-on ("_package_JoyRef"))
    (:file "_package_JoyRef" :depends-on ("_package"))
    (:file "MagPose" :depends-on ("_package_MagPose"))
    (:file "_package_MagPose" :depends-on ("_package"))
    (:file "RoboStates" :depends-on ("_package_RoboStates"))
    (:file "_package_RoboStates" :depends-on ("_package"))
  ))
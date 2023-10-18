
(cl:in-package :asdf)

(defsystem "magmed_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetRobotState" :depends-on ("_package_GetRobotState"))
    (:file "_package_GetRobotState" :depends-on ("_package"))
  ))
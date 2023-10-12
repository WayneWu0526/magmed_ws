
(cl:in-package :asdf)

(defsystem "magmed_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CtrlTwist" :depends-on ("_package_CtrlTwist"))
    (:file "_package_CtrlTwist" :depends-on ("_package"))
  ))
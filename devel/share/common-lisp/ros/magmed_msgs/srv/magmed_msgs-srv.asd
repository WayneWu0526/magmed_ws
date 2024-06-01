
(cl:in-package :asdf)

(defsystem "magmed_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :magmed_msgs-msg
)
  :components ((:file "_package")
    (:file "SelfCollisionCheck" :depends-on ("_package_SelfCollisionCheck"))
    (:file "_package_SelfCollisionCheck" :depends-on ("_package"))
  ))
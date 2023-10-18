;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::RoboStates)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'RoboStates (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::ROBOSTATES")
  (make-package "MAGMED_MSGS::ROBOSTATES"))

(in-package "ROS")
;;//! \htmlinclude RoboStates.msg.html


(intern "*INIT*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*INIT* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*INIT* 0)
(intern "*RUN*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*RUN* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*RUN* 1)
(intern "*TERM*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*TERM* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*TERM* -1)

(defun magmed_msgs::RoboStates-to-symbol (const)
  (cond
        ((= const 0) 'magmed_msgs::RoboStates::*INIT*)
        ((= const 1) 'magmed_msgs::RoboStates::*RUN*)
        ((= const -1) 'magmed_msgs::RoboStates::*TERM*)
        (t nil)))

(defclass magmed_msgs::RoboStates
  :super ros::object
  :slots ())

(defmethod magmed_msgs::RoboStates
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get magmed_msgs::RoboStates :md5sum-) "f0c86d6612d2dc3f3483b4c4ba196520")
(setf (get magmed_msgs::RoboStates :datatype-) "magmed_msgs/RoboStates")
(setf (get magmed_msgs::RoboStates :definition-)
      "int32  INIT = 0    # 初始化状态
int32  RUN = 1  # 运行状态
int32  TERM = -1    # 终止状态
")



(provide :magmed_msgs/RoboStates "f0c86d6612d2dc3f3483b4c4ba196520")



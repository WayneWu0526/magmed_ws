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
  :slots (_VAL ))

(defmethod magmed_msgs::RoboStates
  (:init
   (&key
    ((:VAL __VAL) 0)
    )
   (send-super :init)
   (setq _VAL (round __VAL))
   self)
  (:VAL
   (&optional __VAL)
   (if __VAL (setq _VAL __VAL)) _VAL)
  (:serialization-length
   ()
   (+
    ;; int32 _VAL
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _VAL
       (write-long _VAL s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _VAL
     (setq _VAL (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get magmed_msgs::RoboStates :md5sum-) "3f7db48bba6b67d991a886a8d887cb31")
(setf (get magmed_msgs::RoboStates :datatype-) "magmed_msgs/RoboStates")
(setf (get magmed_msgs::RoboStates :definition-)
      "int32  INIT = 0    # 初始化状态
int32  RUN = 1  # 运行状态
int32  TERM = -1    # 终止状态
int32 VAL
")



(provide :magmed_msgs/RoboStates "3f7db48bba6b67d991a886a8d887cb31")



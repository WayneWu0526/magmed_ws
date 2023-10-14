;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::RoboStates)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'RoboStates (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::ROBOSTATES")
  (make-package "MAGMED_MSGS::ROBOSTATES"))

(in-package "ROS")
;;//! \htmlinclude RoboStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*INIT*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*INIT* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*INIT* 0)
(intern "*CTRL*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*CTRL* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*CTRL* 1)
(intern "*TERM*" (find-package "MAGMED_MSGS::ROBOSTATES"))
(shadow '*TERM* (find-package "MAGMED_MSGS::ROBOSTATES"))
(defconstant magmed_msgs::RoboStates::*TERM* 2)

(defun magmed_msgs::RoboStates-to-symbol (const)
  (cond
        ((= const 0) 'magmed_msgs::RoboStates::*INIT*)
        ((= const 1) 'magmed_msgs::RoboStates::*CTRL*)
        ((= const 2) 'magmed_msgs::RoboStates::*TERM*)
        (t nil)))

(defclass magmed_msgs::RoboStates
  :super ros::object
  :slots (_header ))

(defmethod magmed_msgs::RoboStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    )
   (send-super :init)
   (setq _header __header)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;;
   self)
  )

(setf (get magmed_msgs::RoboStates :md5sum-) "4437f8da4929f4c392a982caafd44e62")
(setf (get magmed_msgs::RoboStates :datatype-) "magmed_msgs/RoboStates")
(setf (get magmed_msgs::RoboStates :definition-)
      "Header header
int8  INIT = 0    # 初始化状态
int8  CTRL = 1    # 控制状态
int8  TERM = 2    # 终止状态
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :magmed_msgs/RoboStates "4437f8da4929f4c392a982caafd44e62")



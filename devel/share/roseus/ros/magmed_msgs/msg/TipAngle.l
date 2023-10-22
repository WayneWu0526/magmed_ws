;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::TipAngle)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'TipAngle (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::TIPANGLE")
  (make-package "MAGMED_MSGS::TIPANGLE"))

(in-package "ROS")
;;//! \htmlinclude TipAngle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass magmed_msgs::TipAngle
  :super ros::object
  :slots (_header _tipAngle ))

(defmethod magmed_msgs::TipAngle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tipAngle __tipAngle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _tipAngle (float __tipAngle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tipAngle
   (&optional __tipAngle)
   (if __tipAngle (setq _tipAngle __tipAngle)) _tipAngle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _tipAngle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _tipAngle
       (sys::poke _tipAngle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _tipAngle
     (setq _tipAngle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get magmed_msgs::TipAngle :md5sum-) "53e2c53db502ae3cd8c298da558777c3")
(setf (get magmed_msgs::TipAngle :datatype-) "magmed_msgs/TipAngle")
(setf (get magmed_msgs::TipAngle :definition-)
      "Header header
float64 tipAngle
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



(provide :magmed_msgs/TipAngle "53e2c53db502ae3cd8c298da558777c3")



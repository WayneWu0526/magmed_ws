;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::JoyRef)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'JoyRef (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::JOYREF")
  (make-package "MAGMED_MSGS::JOYREF"))

(in-package "ROS")
;;//! \htmlinclude JoyRef.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass magmed_msgs::JoyRef
  :super ros::object
  :slots (_header _theta _thetadot _phi _phidot ))

(defmethod magmed_msgs::JoyRef
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:theta __theta) 0.0)
    ((:thetadot __thetadot) 0.0)
    ((:phi __phi) 0.0)
    ((:phidot __phidot) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _theta (float __theta))
   (setq _thetadot (float __thetadot))
   (setq _phi (float __phi))
   (setq _phidot (float __phidot))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:thetadot
   (&optional __thetadot)
   (if __thetadot (setq _thetadot __thetadot)) _thetadot)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:phidot
   (&optional __phidot)
   (if __phidot (setq _phidot __phidot)) _phidot)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _theta
    8
    ;; float64 _thetadot
    8
    ;; float64 _phi
    8
    ;; float64 _phidot
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thetadot
       (sys::poke _thetadot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _phidot
       (sys::poke _phidot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thetadot
     (setq _thetadot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _phi
     (setq _phi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _phidot
     (setq _phidot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get magmed_msgs::JoyRef :md5sum-) "aa9cd8cd2e3cd0a81e9dae62d67c6a55")
(setf (get magmed_msgs::JoyRef :datatype-) "magmed_msgs/JoyRef")
(setf (get magmed_msgs::JoyRef :definition-)
      "Header header
float64 theta
float64 thetadot
float64 phi
float64 phidot
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



(provide :magmed_msgs/JoyRef "aa9cd8cd2e3cd0a81e9dae62d67c6a55")



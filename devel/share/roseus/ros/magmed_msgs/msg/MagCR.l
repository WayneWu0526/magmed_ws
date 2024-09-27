;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::MagCR)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'MagCR (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::MAGCR")
  (make-package "MAGMED_MSGS::MAGCR"))

(in-package "ROS")
;;//! \htmlinclude MagCR.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass magmed_msgs::MagCR
  :super ros::object
  :slots (_header _theta _phi _Tsg ))

(defmethod magmed_msgs::MagCR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:theta __theta) 0.0)
    ((:phi __phi) 0.0)
    ((:Tsg __Tsg) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _theta (float __theta))
   (setq _phi (float __phi))
   (setq _Tsg __Tsg)
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
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:Tsg
   (&rest __Tsg)
   (if (keywordp (car __Tsg))
       (send* _Tsg __Tsg)
     (progn
       (if __Tsg (setq _Tsg (car __Tsg)))
       _Tsg)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _theta
    8
    ;; float64 _phi
    8
    ;; geometry_msgs/Pose _Tsg
    (send _Tsg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _Tsg
       (send _Tsg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _phi
     (setq _phi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _Tsg
     (send _Tsg :deserialize buf ptr-) (incf ptr- (send _Tsg :serialization-length))
   ;;
   self)
  )

(setf (get magmed_msgs::MagCR :md5sum-) "18eb392c0287f61819aca93877ff35f7")
(setf (get magmed_msgs::MagCR :datatype-) "magmed_msgs/MagCR")
(setf (get magmed_msgs::MagCR :definition-)
      "Header header
float64 theta
float64 phi
geometry_msgs/Pose Tsg
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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :magmed_msgs/MagCR "18eb392c0287f61819aca93877ff35f7")



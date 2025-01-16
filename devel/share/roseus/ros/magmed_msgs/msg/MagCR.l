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
  :slots (_header _phi_mock _thetaL_mock _phi_msr _thetaL_msr _tipPoint _Tsg ))

(defmethod magmed_msgs::MagCR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:phi_mock __phi_mock) 0.0)
    ((:thetaL_mock __thetaL_mock) 0.0)
    ((:phi_msr __phi_msr) 0.0)
    ((:thetaL_msr __thetaL_msr) 0.0)
    ((:tipPoint __tipPoint) (instance geometry_msgs::Point :init))
    ((:Tsg __Tsg) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _phi_mock (float __phi_mock))
   (setq _thetaL_mock (float __thetaL_mock))
   (setq _phi_msr (float __phi_msr))
   (setq _thetaL_msr (float __thetaL_msr))
   (setq _tipPoint __tipPoint)
   (setq _Tsg __Tsg)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:phi_mock
   (&optional __phi_mock)
   (if __phi_mock (setq _phi_mock __phi_mock)) _phi_mock)
  (:thetaL_mock
   (&optional __thetaL_mock)
   (if __thetaL_mock (setq _thetaL_mock __thetaL_mock)) _thetaL_mock)
  (:phi_msr
   (&optional __phi_msr)
   (if __phi_msr (setq _phi_msr __phi_msr)) _phi_msr)
  (:thetaL_msr
   (&optional __thetaL_msr)
   (if __thetaL_msr (setq _thetaL_msr __thetaL_msr)) _thetaL_msr)
  (:tipPoint
   (&rest __tipPoint)
   (if (keywordp (car __tipPoint))
       (send* _tipPoint __tipPoint)
     (progn
       (if __tipPoint (setq _tipPoint (car __tipPoint)))
       _tipPoint)))
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
    ;; float64 _phi_mock
    8
    ;; float64 _thetaL_mock
    8
    ;; float64 _phi_msr
    8
    ;; float64 _thetaL_msr
    8
    ;; geometry_msgs/Point _tipPoint
    (send _tipPoint :serialization-length)
    ;; geometry_msgs/Pose _Tsg
    (send _Tsg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _phi_mock
       (sys::poke _phi_mock (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thetaL_mock
       (sys::poke _thetaL_mock (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _phi_msr
       (sys::poke _phi_msr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thetaL_msr
       (sys::poke _thetaL_msr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _tipPoint
       (send _tipPoint :serialize s)
     ;; geometry_msgs/Pose _Tsg
       (send _Tsg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _phi_mock
     (setq _phi_mock (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thetaL_mock
     (setq _thetaL_mock (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _phi_msr
     (setq _phi_msr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thetaL_msr
     (setq _thetaL_msr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _tipPoint
     (send _tipPoint :deserialize buf ptr-) (incf ptr- (send _tipPoint :serialization-length))
   ;; geometry_msgs/Pose _Tsg
     (send _Tsg :deserialize buf ptr-) (incf ptr- (send _Tsg :serialization-length))
   ;;
   self)
  )

(setf (get magmed_msgs::MagCR :md5sum-) "d2b6af8e88cbe5faeaf2769acfe589ea")
(setf (get magmed_msgs::MagCR :datatype-) "magmed_msgs/MagCR")
(setf (get magmed_msgs::MagCR :definition-)
      "Header header

float64 phi_mock
float64 thetaL_mock
float64 phi_msr
float64 thetaL_msr
geometry_msgs/Point tipPoint
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :magmed_msgs/MagCR "d2b6af8e88cbe5faeaf2769acfe589ea")



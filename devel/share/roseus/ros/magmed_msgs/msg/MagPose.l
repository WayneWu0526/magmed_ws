;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::MagPose)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'MagPose (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::MAGPOSE")
  (make-package "MAGMED_MSGS::MAGPOSE"))

(in-package "ROS")
;;//! \htmlinclude MagPose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass magmed_msgs::MagPose
  :super ros::object
  :slots (_header _psi _pos ))

(defmethod magmed_msgs::MagPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:psi __psi) 0.0)
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _psi (float __psi))
   (setq _pos __pos)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _psi
    8
    ;; float64[3] _pos
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _pos
     (dotimes (i 3)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get magmed_msgs::MagPose :md5sum-) "8a3b15d1fb390ff3d6d18f2e1b4aa606")
(setf (get magmed_msgs::MagPose :datatype-) "magmed_msgs/MagPose")
(setf (get magmed_msgs::MagPose :definition-)
      "Header header
float64 psi
float64[3] pos
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



(provide :magmed_msgs/MagPose "8a3b15d1fb390ff3d6d18f2e1b4aa606")



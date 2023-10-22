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
  :slots (_header _refPhi _refTheta ))

(defmethod magmed_msgs::JoyRef
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:refPhi __refPhi) (instance magmed_msgs::RefPhi :init))
    ((:refTheta __refTheta) (instance magmed_msgs::RefTheta :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _refPhi __refPhi)
   (setq _refTheta __refTheta)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:refPhi
   (&rest __refPhi)
   (if (keywordp (car __refPhi))
       (send* _refPhi __refPhi)
     (progn
       (if __refPhi (setq _refPhi (car __refPhi)))
       _refPhi)))
  (:refTheta
   (&rest __refTheta)
   (if (keywordp (car __refTheta))
       (send* _refTheta __refTheta)
     (progn
       (if __refTheta (setq _refTheta (car __refTheta)))
       _refTheta)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; magmed_msgs/RefPhi _refPhi
    (send _refPhi :serialization-length)
    ;; magmed_msgs/RefTheta _refTheta
    (send _refTheta :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; magmed_msgs/RefPhi _refPhi
       (send _refPhi :serialize s)
     ;; magmed_msgs/RefTheta _refTheta
       (send _refTheta :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; magmed_msgs/RefPhi _refPhi
     (send _refPhi :deserialize buf ptr-) (incf ptr- (send _refPhi :serialization-length))
   ;; magmed_msgs/RefTheta _refTheta
     (send _refTheta :deserialize buf ptr-) (incf ptr- (send _refTheta :serialization-length))
   ;;
   self)
  )

(setf (get magmed_msgs::JoyRef :md5sum-) "ea53e564f4388a7ea7a788d618611b29")
(setf (get magmed_msgs::JoyRef :datatype-) "magmed_msgs/JoyRef")
(setf (get magmed_msgs::JoyRef :definition-)
      "Header header
RefPhi refPhi 
RefTheta refTheta
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
MSG: magmed_msgs/RefPhi
float64 phi
float64 dphi
================================================================================
MSG: magmed_msgs/RefTheta
float64 theta
float64 dtheta
")



(provide :magmed_msgs/JoyRef "ea53e564f4388a7ea7a788d618611b29")



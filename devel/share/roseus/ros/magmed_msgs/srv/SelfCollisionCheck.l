;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::SelfCollisionCheck)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'SelfCollisionCheck (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::SELFCOLLISIONCHECK")
  (make-package "MAGMED_MSGS::SELFCOLLISIONCHECK"))
(unless (find-package "MAGMED_MSGS::SELFCOLLISIONCHECKREQUEST")
  (make-package "MAGMED_MSGS::SELFCOLLISIONCHECKREQUEST"))
(unless (find-package "MAGMED_MSGS::SELFCOLLISIONCHECKRESPONSE")
  (make-package "MAGMED_MSGS::SELFCOLLISIONCHECKRESPONSE"))

(in-package "ROS")





(defclass magmed_msgs::SelfCollisionCheckRequest
  :super ros::object
  :slots (_joints ))

(defmethod magmed_msgs::SelfCollisionCheckRequest
  (:init
   (&key
    ((:joints __joints) (instance magmed_msgs::RoboJoints :init))
    )
   (send-super :init)
   (setq _joints __joints)
   self)
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:serialization-length
   ()
   (+
    ;; magmed_msgs/RoboJoints _joints
    (send _joints :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; magmed_msgs/RoboJoints _joints
       (send _joints :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; magmed_msgs/RoboJoints _joints
     (send _joints :deserialize buf ptr-) (incf ptr- (send _joints :serialization-length))
   ;;
   self)
  )

(defclass magmed_msgs::SelfCollisionCheckResponse
  :super ros::object
  :slots (_checkResult ))

(defmethod magmed_msgs::SelfCollisionCheckResponse
  (:init
   (&key
    ((:checkResult __checkResult) nil)
    )
   (send-super :init)
   (setq _checkResult __checkResult)
   self)
  (:checkResult
   (&optional (__checkResult :null))
   (if (not (eq __checkResult :null)) (setq _checkResult __checkResult)) _checkResult)
  (:serialization-length
   ()
   (+
    ;; bool _checkResult
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _checkResult
       (if _checkResult (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _checkResult
     (setq _checkResult (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass magmed_msgs::SelfCollisionCheck
  :super ros::object
  :slots ())

(setf (get magmed_msgs::SelfCollisionCheck :md5sum-) "7060f063846e94f79170ebac9812a07a")
(setf (get magmed_msgs::SelfCollisionCheck :datatype-) "magmed_msgs/SelfCollisionCheck")
(setf (get magmed_msgs::SelfCollisionCheck :request) magmed_msgs::SelfCollisionCheckRequest)
(setf (get magmed_msgs::SelfCollisionCheck :response) magmed_msgs::SelfCollisionCheckResponse)

(defmethod magmed_msgs::SelfCollisionCheckRequest
  (:response () (instance magmed_msgs::SelfCollisionCheckResponse :init)))

(setf (get magmed_msgs::SelfCollisionCheckRequest :md5sum-) "7060f063846e94f79170ebac9812a07a")
(setf (get magmed_msgs::SelfCollisionCheckRequest :datatype-) "magmed_msgs/SelfCollisionCheckRequest")
(setf (get magmed_msgs::SelfCollisionCheckRequest :definition-)
      "RoboJoints joints

================================================================================
MSG: magmed_msgs/RoboJoints
Header header
float64[] joints
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
---
bool checkResult
")

(setf (get magmed_msgs::SelfCollisionCheckResponse :md5sum-) "7060f063846e94f79170ebac9812a07a")
(setf (get magmed_msgs::SelfCollisionCheckResponse :datatype-) "magmed_msgs/SelfCollisionCheckResponse")
(setf (get magmed_msgs::SelfCollisionCheckResponse :definition-)
      "RoboJoints joints

================================================================================
MSG: magmed_msgs/RoboJoints
Header header
float64[] joints
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
---
bool checkResult
")



(provide :magmed_msgs/SelfCollisionCheck "7060f063846e94f79170ebac9812a07a")



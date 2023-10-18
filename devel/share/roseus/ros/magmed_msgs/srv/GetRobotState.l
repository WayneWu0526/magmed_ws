;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::GetRobotState)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'GetRobotState (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::GETROBOTSTATE")
  (make-package "MAGMED_MSGS::GETROBOTSTATE"))
(unless (find-package "MAGMED_MSGS::GETROBOTSTATEREQUEST")
  (make-package "MAGMED_MSGS::GETROBOTSTATEREQUEST"))
(unless (find-package "MAGMED_MSGS::GETROBOTSTATERESPONSE")
  (make-package "MAGMED_MSGS::GETROBOTSTATERESPONSE"))

(in-package "ROS")





(defclass magmed_msgs::GetRobotStateRequest
  :super ros::object
  :slots (_request ))

(defmethod magmed_msgs::GetRobotStateRequest
  (:init
   (&key
    ((:request __request) 0)
    )
   (send-super :init)
   (setq _request (round __request))
   self)
  (:request
   (&optional __request)
   (if __request (setq _request __request)) _request)
  (:serialization-length
   ()
   (+
    ;; int32 _request
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _request
       (write-long _request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _request
     (setq _request (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass magmed_msgs::GetRobotStateResponse
  :super ros::object
  :slots (_response ))

(defmethod magmed_msgs::GetRobotStateResponse
  (:init
   (&key
    ((:response __response) 0)
    )
   (send-super :init)
   (setq _response (round __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; int32 _response
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _response
       (write-long _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _response
     (setq _response (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass magmed_msgs::GetRobotState
  :super ros::object
  :slots ())

(setf (get magmed_msgs::GetRobotState :md5sum-) "51edd9dfd50014fde2b589cbf77706aa")
(setf (get magmed_msgs::GetRobotState :datatype-) "magmed_msgs/GetRobotState")
(setf (get magmed_msgs::GetRobotState :request) magmed_msgs::GetRobotStateRequest)
(setf (get magmed_msgs::GetRobotState :response) magmed_msgs::GetRobotStateResponse)

(defmethod magmed_msgs::GetRobotStateRequest
  (:response () (instance magmed_msgs::GetRobotStateResponse :init)))

(setf (get magmed_msgs::GetRobotStateRequest :md5sum-) "51edd9dfd50014fde2b589cbf77706aa")
(setf (get magmed_msgs::GetRobotStateRequest :datatype-) "magmed_msgs/GetRobotStateRequest")
(setf (get magmed_msgs::GetRobotStateRequest :definition-)
      "# GetRobotState.srv
int32 request
---
int32 response
")

(setf (get magmed_msgs::GetRobotStateResponse :md5sum-) "51edd9dfd50014fde2b589cbf77706aa")
(setf (get magmed_msgs::GetRobotStateResponse :datatype-) "magmed_msgs/GetRobotStateResponse")
(setf (get magmed_msgs::GetRobotStateResponse :definition-)
      "# GetRobotState.srv
int32 request
---
int32 response
")



(provide :magmed_msgs/GetRobotState "51edd9dfd50014fde2b589cbf77706aa")



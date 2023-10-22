;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::RefTheta)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'RefTheta (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::REFTHETA")
  (make-package "MAGMED_MSGS::REFTHETA"))

(in-package "ROS")
;;//! \htmlinclude RefTheta.msg.html


(defclass magmed_msgs::RefTheta
  :super ros::object
  :slots (_theta _dtheta ))

(defmethod magmed_msgs::RefTheta
  (:init
   (&key
    ((:theta __theta) 0.0)
    ((:dtheta __dtheta) 0.0)
    )
   (send-super :init)
   (setq _theta (float __theta))
   (setq _dtheta (float __dtheta))
   self)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:dtheta
   (&optional __dtheta)
   (if __dtheta (setq _dtheta __dtheta)) _dtheta)
  (:serialization-length
   ()
   (+
    ;; float64 _theta
    8
    ;; float64 _dtheta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dtheta
       (sys::poke _dtheta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dtheta
     (setq _dtheta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get magmed_msgs::RefTheta :md5sum-) "b4963a90c97bbf7fdb1b9c9e9a7d1576")
(setf (get magmed_msgs::RefTheta :datatype-) "magmed_msgs/RefTheta")
(setf (get magmed_msgs::RefTheta :definition-)
      "float64 theta
float64 dtheta
")



(provide :magmed_msgs/RefTheta "b4963a90c97bbf7fdb1b9c9e9a7d1576")



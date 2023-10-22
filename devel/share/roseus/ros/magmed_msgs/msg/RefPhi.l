;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::RefPhi)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'RefPhi (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::REFPHI")
  (make-package "MAGMED_MSGS::REFPHI"))

(in-package "ROS")
;;//! \htmlinclude RefPhi.msg.html


(defclass magmed_msgs::RefPhi
  :super ros::object
  :slots (_phi _dphi ))

(defmethod magmed_msgs::RefPhi
  (:init
   (&key
    ((:phi __phi) 0.0)
    ((:dphi __dphi) 0.0)
    )
   (send-super :init)
   (setq _phi (float __phi))
   (setq _dphi (float __dphi))
   self)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:dphi
   (&optional __dphi)
   (if __dphi (setq _dphi __dphi)) _dphi)
  (:serialization-length
   ()
   (+
    ;; float64 _phi
    8
    ;; float64 _dphi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dphi
       (sys::poke _dphi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _phi
     (setq _phi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dphi
     (setq _dphi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get magmed_msgs::RefPhi :md5sum-) "55e8d7deb213bddd90abc0b338391bbc")
(setf (get magmed_msgs::RefPhi :datatype-) "magmed_msgs/RefPhi")
(setf (get magmed_msgs::RefPhi :definition-)
      "float64 phi
float64 dphi
")



(provide :magmed_msgs/RefPhi "55e8d7deb213bddd90abc0b338391bbc")



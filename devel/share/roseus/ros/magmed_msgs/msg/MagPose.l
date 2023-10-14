;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::MagPose)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'MagPose (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::MAGPOSE")
  (make-package "MAGMED_MSGS::MAGPOSE"))

(in-package "ROS")
;;//! \htmlinclude MagPose.msg.html


(defclass magmed_msgs::MagPose
  :super ros::object
  :slots (_psi _pos ))

(defmethod magmed_msgs::MagPose
  (:init
   (&key
    ((:psi __psi) 0.0)
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _psi (float __psi))
   (setq _pos __pos)
   self)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; float64 _psi
    8
    ;; float64[3] _pos
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get magmed_msgs::MagPose :md5sum-) "ad8b136153ecd61f300fa21020162f97")
(setf (get magmed_msgs::MagPose :datatype-) "magmed_msgs/MagPose")
(setf (get magmed_msgs::MagPose :definition-)
      "float64 psi
float64[3] pos
")



(provide :magmed_msgs/MagPose "ad8b136153ecd61f300fa21020162f97")



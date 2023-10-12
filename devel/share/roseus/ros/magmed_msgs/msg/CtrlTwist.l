;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::CtrlTwist)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'CtrlTwist (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::CTRLTWIST")
  (make-package "MAGMED_MSGS::CTRLTWIST"))

(in-package "ROS")
;;//! \htmlinclude CtrlTwist.msg.html


(defclass magmed_msgs::CtrlTwist
  :super ros::object
  :slots (_psi _p ))

(defmethod magmed_msgs::CtrlTwist
  (:init
   (&key
    ((:psi __psi) 0.0)
    ((:p __p) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _psi (float __psi))
   (setq _p __p)
   self)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:p
   (&optional __p)
   (if __p (setq _p __p)) _p)
  (:serialization-length
   ()
   (+
    ;; float64 _psi
    8
    ;; float64[3] _p
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _p
     (dotimes (i 3)
       (sys::poke (elt _p i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _p
   (dotimes (i (length _p))
     (setf (elt _p i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get magmed_msgs::CtrlTwist :md5sum-) "76917cdec57096cc983b4e09ed694109")
(setf (get magmed_msgs::CtrlTwist :datatype-) "magmed_msgs/CtrlTwist")
(setf (get magmed_msgs::CtrlTwist :definition-)
      "float64 psi
float64[3] p
")



(provide :magmed_msgs/CtrlTwist "76917cdec57096cc983b4e09ed694109")



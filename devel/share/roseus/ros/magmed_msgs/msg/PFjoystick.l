;; Auto-generated. Do not edit!


(when (boundp 'magmed_msgs::PFjoystick)
  (if (not (find-package "MAGMED_MSGS"))
    (make-package "MAGMED_MSGS"))
  (shadow 'PFjoystick (find-package "MAGMED_MSGS")))
(unless (find-package "MAGMED_MSGS::PFJOYSTICK")
  (make-package "MAGMED_MSGS::PFJOYSTICK"))

(in-package "ROS")
;;//! \htmlinclude PFjoystick.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass magmed_msgs::PFjoystick
  :super ros::object
  :slots (_header _nJOY1 _nJOY2 _nJOY3 _bJOYD _POTA _POTB _BANA _BANB _ENCA _ENCB _TOG _BUT ))

(defmethod magmed_msgs::PFjoystick
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:nJOY1 __nJOY1) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:nJOY2 __nJOY2) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:nJOY3 __nJOY3) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:bJOYD __bJOYD) nil)
    ((:POTA __POTA) 0)
    ((:POTB __POTB) 0)
    ((:BANA __BANA) 0)
    ((:BANB __BANB) 0)
    ((:ENCA __ENCA) 0)
    ((:ENCB __ENCB) 0)
    ((:TOG __TOG) (let (r) (dotimes (i 5) (push nil r)) r))
    ((:BUT __BUT) (let (r) (dotimes (i 6) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _nJOY1 __nJOY1)
   (setq _nJOY2 __nJOY2)
   (setq _nJOY3 __nJOY3)
   (setq _bJOYD __bJOYD)
   (setq _POTA (round __POTA))
   (setq _POTB (round __POTB))
   (setq _BANA (round __BANA))
   (setq _BANB (round __BANB))
   (setq _ENCA (round __ENCA))
   (setq _ENCB (round __ENCB))
   (setq _TOG __TOG)
   (setq _BUT __BUT)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:nJOY1
   (&optional __nJOY1)
   (if __nJOY1 (setq _nJOY1 __nJOY1)) _nJOY1)
  (:nJOY2
   (&optional __nJOY2)
   (if __nJOY2 (setq _nJOY2 __nJOY2)) _nJOY2)
  (:nJOY3
   (&optional __nJOY3)
   (if __nJOY3 (setq _nJOY3 __nJOY3)) _nJOY3)
  (:bJOYD
   (&optional (__bJOYD :null))
   (if (not (eq __bJOYD :null)) (setq _bJOYD __bJOYD)) _bJOYD)
  (:POTA
   (&optional __POTA)
   (if __POTA (setq _POTA __POTA)) _POTA)
  (:POTB
   (&optional __POTB)
   (if __POTB (setq _POTB __POTB)) _POTB)
  (:BANA
   (&optional __BANA)
   (if __BANA (setq _BANA __BANA)) _BANA)
  (:BANB
   (&optional __BANB)
   (if __BANB (setq _BANB __BANB)) _BANB)
  (:ENCA
   (&optional __ENCA)
   (if __ENCA (setq _ENCA __ENCA)) _ENCA)
  (:ENCB
   (&optional __ENCB)
   (if __ENCB (setq _ENCB __ENCB)) _ENCB)
  (:TOG
   (&optional __TOG)
   (if __TOG (setq _TOG __TOG)) _TOG)
  (:BUT
   (&optional __BUT)
   (if __BUT (setq _BUT __BUT)) _BUT)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[3] _nJOY1
    (* 4    3)
    ;; float32[2] _nJOY2
    (* 4    2)
    ;; float32[2] _nJOY3
    (* 4    2)
    ;; bool _bJOYD
    1
    ;; uint16 _POTA
    2
    ;; uint16 _POTB
    2
    ;; int32 _BANA
    4
    ;; int32 _BANB
    4
    ;; int16 _ENCA
    2
    ;; int16 _ENCB
    2
    ;; bool[5] _TOG
    (* 1    5)
    ;; bool[6] _BUT
    (* 1    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[3] _nJOY1
     (dotimes (i 3)
       (sys::poke (elt _nJOY1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _nJOY2
     (dotimes (i 2)
       (sys::poke (elt _nJOY2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _nJOY3
     (dotimes (i 2)
       (sys::poke (elt _nJOY3 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _bJOYD
       (if _bJOYD (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _POTA
       (write-word _POTA s)
     ;; uint16 _POTB
       (write-word _POTB s)
     ;; int32 _BANA
       (write-long _BANA s)
     ;; int32 _BANB
       (write-long _BANB s)
     ;; int16 _ENCA
       (write-word _ENCA s)
     ;; int16 _ENCB
       (write-word _ENCB s)
     ;; bool[5] _TOG
     (dotimes (i 5)
       (if (elt _TOG i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[6] _BUT
     (dotimes (i 6)
       (if (elt _BUT i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[3] _nJOY1
   (dotimes (i (length _nJOY1))
     (setf (elt _nJOY1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _nJOY2
   (dotimes (i (length _nJOY2))
     (setf (elt _nJOY2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _nJOY3
   (dotimes (i (length _nJOY3))
     (setf (elt _nJOY3 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; bool _bJOYD
     (setq _bJOYD (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _POTA
     (setq _POTA (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _POTB
     (setq _POTB (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _BANA
     (setq _BANA (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _BANB
     (setq _BANB (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _ENCA
     (setq _ENCA (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _ENCB
     (setq _ENCB (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool[5] _TOG
   (dotimes (i (length _TOG))
     (setf (elt _TOG i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[6] _BUT
   (dotimes (i (length _BUT))
     (setf (elt _BUT i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get magmed_msgs::PFjoystick :md5sum-) "c1b2838b51e4cc36d6636da93093d28d")
(setf (get magmed_msgs::PFjoystick :datatype-) "magmed_msgs/PFjoystick")
(setf (get magmed_msgs::PFjoystick :definition-)
      "Header header              # ROS standard header

float32[3] nJOY1             # Three axes of the big joystick
float32[2] nJOY2             # Two axes of the first (left) small joystick
float32[2] nJOY3             # Two axes of the second (right) small joystick
bool bJOYD                 # Big joystick button
uint16 POTA                # Potentiometer A
uint16 POTB                # Potentiometer B
int32 BANA                 # Rotary switch A
int32 BANB                 # Rotary switch B
int16 ENCA                 # Encoder A
int16 ENCB                 # Encoder B
bool[5] TOG                # Toggle switches (5 in total)
bool[6] BUT                # Push buttons (6 in total)

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



(provide :magmed_msgs/PFjoystick "c1b2838b51e4cc36d6636da93093d28d")



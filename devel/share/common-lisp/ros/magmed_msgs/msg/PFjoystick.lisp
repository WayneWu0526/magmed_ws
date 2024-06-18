; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude PFjoystick.msg.html

(cl:defclass <PFjoystick> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (nJOY1
    :reader nJOY1
    :initarg :nJOY1
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (nJOY2
    :reader nJOY2
    :initarg :nJOY2
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (nJOY3
    :reader nJOY3
    :initarg :nJOY3
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (bJOYD
    :reader bJOYD
    :initarg :bJOYD
    :type cl:boolean
    :initform cl:nil)
   (POTA
    :reader POTA
    :initarg :POTA
    :type cl:fixnum
    :initform 0)
   (POTB
    :reader POTB
    :initarg :POTB
    :type cl:fixnum
    :initform 0)
   (BANA
    :reader BANA
    :initarg :BANA
    :type cl:integer
    :initform 0)
   (BANB
    :reader BANB
    :initarg :BANB
    :type cl:integer
    :initform 0)
   (ENCA
    :reader ENCA
    :initarg :ENCA
    :type cl:fixnum
    :initform 0)
   (ENCB
    :reader ENCB
    :initarg :ENCB
    :type cl:fixnum
    :initform 0)
   (TOG
    :reader TOG
    :initarg :TOG
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 5 :element-type 'cl:boolean :initial-element cl:nil))
   (BUT
    :reader BUT
    :initarg :BUT
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 6 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass PFjoystick (<PFjoystick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PFjoystick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PFjoystick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<PFjoystick> is deprecated: use magmed_msgs-msg:PFjoystick instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:header-val is deprecated.  Use magmed_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'nJOY1-val :lambda-list '(m))
(cl:defmethod nJOY1-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:nJOY1-val is deprecated.  Use magmed_msgs-msg:nJOY1 instead.")
  (nJOY1 m))

(cl:ensure-generic-function 'nJOY2-val :lambda-list '(m))
(cl:defmethod nJOY2-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:nJOY2-val is deprecated.  Use magmed_msgs-msg:nJOY2 instead.")
  (nJOY2 m))

(cl:ensure-generic-function 'nJOY3-val :lambda-list '(m))
(cl:defmethod nJOY3-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:nJOY3-val is deprecated.  Use magmed_msgs-msg:nJOY3 instead.")
  (nJOY3 m))

(cl:ensure-generic-function 'bJOYD-val :lambda-list '(m))
(cl:defmethod bJOYD-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:bJOYD-val is deprecated.  Use magmed_msgs-msg:bJOYD instead.")
  (bJOYD m))

(cl:ensure-generic-function 'POTA-val :lambda-list '(m))
(cl:defmethod POTA-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:POTA-val is deprecated.  Use magmed_msgs-msg:POTA instead.")
  (POTA m))

(cl:ensure-generic-function 'POTB-val :lambda-list '(m))
(cl:defmethod POTB-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:POTB-val is deprecated.  Use magmed_msgs-msg:POTB instead.")
  (POTB m))

(cl:ensure-generic-function 'BANA-val :lambda-list '(m))
(cl:defmethod BANA-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:BANA-val is deprecated.  Use magmed_msgs-msg:BANA instead.")
  (BANA m))

(cl:ensure-generic-function 'BANB-val :lambda-list '(m))
(cl:defmethod BANB-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:BANB-val is deprecated.  Use magmed_msgs-msg:BANB instead.")
  (BANB m))

(cl:ensure-generic-function 'ENCA-val :lambda-list '(m))
(cl:defmethod ENCA-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:ENCA-val is deprecated.  Use magmed_msgs-msg:ENCA instead.")
  (ENCA m))

(cl:ensure-generic-function 'ENCB-val :lambda-list '(m))
(cl:defmethod ENCB-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:ENCB-val is deprecated.  Use magmed_msgs-msg:ENCB instead.")
  (ENCB m))

(cl:ensure-generic-function 'TOG-val :lambda-list '(m))
(cl:defmethod TOG-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:TOG-val is deprecated.  Use magmed_msgs-msg:TOG instead.")
  (TOG m))

(cl:ensure-generic-function 'BUT-val :lambda-list '(m))
(cl:defmethod BUT-val ((m <PFjoystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:BUT-val is deprecated.  Use magmed_msgs-msg:BUT instead.")
  (BUT m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PFjoystick>) ostream)
  "Serializes a message object of type '<PFjoystick>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'nJOY1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'nJOY2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'nJOY3))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'bJOYD) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'POTA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'POTA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'POTB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'POTB)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'BANA)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'BANB)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ENCA)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ENCB)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'TOG))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'BUT))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PFjoystick>) istream)
  "Deserializes a message object of type '<PFjoystick>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'nJOY1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'nJOY1)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'nJOY2) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'nJOY2)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'nJOY3) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'nJOY3)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'bJOYD) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'POTA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'POTA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'POTB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'POTB)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'BANA) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'BANB) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ENCA) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ENCB) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'TOG) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'TOG)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'BUT) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'BUT)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PFjoystick>)))
  "Returns string type for a message object of type '<PFjoystick>"
  "magmed_msgs/PFjoystick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PFjoystick)))
  "Returns string type for a message object of type 'PFjoystick"
  "magmed_msgs/PFjoystick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PFjoystick>)))
  "Returns md5sum for a message object of type '<PFjoystick>"
  "c1b2838b51e4cc36d6636da93093d28d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PFjoystick)))
  "Returns md5sum for a message object of type 'PFjoystick"
  "c1b2838b51e4cc36d6636da93093d28d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PFjoystick>)))
  "Returns full string definition for message of type '<PFjoystick>"
  (cl:format cl:nil "Header header              # ROS standard header~%~%float32[3] nJOY1             # Three axes of the big joystick~%float32[2] nJOY2             # Two axes of the first (left) small joystick~%float32[2] nJOY3             # Two axes of the second (right) small joystick~%bool bJOYD                 # Big joystick button~%uint16 POTA                # Potentiometer A~%uint16 POTB                # Potentiometer B~%int32 BANA                 # Rotary switch A~%int32 BANB                 # Rotary switch B~%int16 ENCA                 # Encoder A~%int16 ENCB                 # Encoder B~%bool[5] TOG                # Toggle switches (5 in total)~%bool[6] BUT                # Push buttons (6 in total)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PFjoystick)))
  "Returns full string definition for message of type 'PFjoystick"
  (cl:format cl:nil "Header header              # ROS standard header~%~%float32[3] nJOY1             # Three axes of the big joystick~%float32[2] nJOY2             # Two axes of the first (left) small joystick~%float32[2] nJOY3             # Two axes of the second (right) small joystick~%bool bJOYD                 # Big joystick button~%uint16 POTA                # Potentiometer A~%uint16 POTB                # Potentiometer B~%int32 BANA                 # Rotary switch A~%int32 BANB                 # Rotary switch B~%int16 ENCA                 # Encoder A~%int16 ENCB                 # Encoder B~%bool[5] TOG                # Toggle switches (5 in total)~%bool[6] BUT                # Push buttons (6 in total)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PFjoystick>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'nJOY1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'nJOY2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'nJOY3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     2
     2
     4
     4
     2
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'TOG) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'BUT) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PFjoystick>))
  "Converts a ROS message object to a list"
  (cl:list 'PFjoystick
    (cl:cons ':header (header msg))
    (cl:cons ':nJOY1 (nJOY1 msg))
    (cl:cons ':nJOY2 (nJOY2 msg))
    (cl:cons ':nJOY3 (nJOY3 msg))
    (cl:cons ':bJOYD (bJOYD msg))
    (cl:cons ':POTA (POTA msg))
    (cl:cons ':POTB (POTB msg))
    (cl:cons ':BANA (BANA msg))
    (cl:cons ':BANB (BANB msg))
    (cl:cons ':ENCA (ENCA msg))
    (cl:cons ':ENCB (ENCB msg))
    (cl:cons ':TOG (TOG msg))
    (cl:cons ':BUT (BUT msg))
))

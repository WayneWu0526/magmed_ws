; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude CtrlTwist.msg.html

(cl:defclass <CtrlTwist> (roslisp-msg-protocol:ros-message)
  ((psi
    :reader psi
    :initarg :psi
    :type cl:float
    :initform 0.0)
   (p
    :reader p
    :initarg :p
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CtrlTwist (<CtrlTwist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CtrlTwist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CtrlTwist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<CtrlTwist> is deprecated: use magmed_msgs-msg:CtrlTwist instead.")))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <CtrlTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:psi-val is deprecated.  Use magmed_msgs-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <CtrlTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:p-val is deprecated.  Use magmed_msgs-msg:p instead.")
  (p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CtrlTwist>) ostream)
  "Serializes a message object of type '<CtrlTwist>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'p))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CtrlTwist>) istream)
  "Deserializes a message object of type '<CtrlTwist>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'p) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'p)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CtrlTwist>)))
  "Returns string type for a message object of type '<CtrlTwist>"
  "magmed_msgs/CtrlTwist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CtrlTwist)))
  "Returns string type for a message object of type 'CtrlTwist"
  "magmed_msgs/CtrlTwist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CtrlTwist>)))
  "Returns md5sum for a message object of type '<CtrlTwist>"
  "76917cdec57096cc983b4e09ed694109")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CtrlTwist)))
  "Returns md5sum for a message object of type 'CtrlTwist"
  "76917cdec57096cc983b4e09ed694109")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CtrlTwist>)))
  "Returns full string definition for message of type '<CtrlTwist>"
  (cl:format cl:nil "float64 psi~%float64[3] p~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CtrlTwist)))
  "Returns full string definition for message of type 'CtrlTwist"
  (cl:format cl:nil "float64 psi~%float64[3] p~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CtrlTwist>))
  (cl:+ 0
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CtrlTwist>))
  "Converts a ROS message object to a list"
  (cl:list 'CtrlTwist
    (cl:cons ':psi (psi msg))
    (cl:cons ':p (p msg))
))

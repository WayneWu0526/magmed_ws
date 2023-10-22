; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude RefTheta.msg.html

(cl:defclass <RefTheta> (roslisp-msg-protocol:ros-message)
  ((theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (dtheta
    :reader dtheta
    :initarg :dtheta
    :type cl:float
    :initform 0.0))
)

(cl:defclass RefTheta (<RefTheta>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RefTheta>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RefTheta)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<RefTheta> is deprecated: use magmed_msgs-msg:RefTheta instead.")))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <RefTheta>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:theta-val is deprecated.  Use magmed_msgs-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'dtheta-val :lambda-list '(m))
(cl:defmethod dtheta-val ((m <RefTheta>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:dtheta-val is deprecated.  Use magmed_msgs-msg:dtheta instead.")
  (dtheta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RefTheta>) ostream)
  "Serializes a message object of type '<RefTheta>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dtheta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RefTheta>) istream)
  "Deserializes a message object of type '<RefTheta>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dtheta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RefTheta>)))
  "Returns string type for a message object of type '<RefTheta>"
  "magmed_msgs/RefTheta")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RefTheta)))
  "Returns string type for a message object of type 'RefTheta"
  "magmed_msgs/RefTheta")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RefTheta>)))
  "Returns md5sum for a message object of type '<RefTheta>"
  "b4963a90c97bbf7fdb1b9c9e9a7d1576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RefTheta)))
  "Returns md5sum for a message object of type 'RefTheta"
  "b4963a90c97bbf7fdb1b9c9e9a7d1576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RefTheta>)))
  "Returns full string definition for message of type '<RefTheta>"
  (cl:format cl:nil "float64 theta~%float64 dtheta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RefTheta)))
  "Returns full string definition for message of type 'RefTheta"
  (cl:format cl:nil "float64 theta~%float64 dtheta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RefTheta>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RefTheta>))
  "Converts a ROS message object to a list"
  (cl:list 'RefTheta
    (cl:cons ':theta (theta msg))
    (cl:cons ':dtheta (dtheta msg))
))

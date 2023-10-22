; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude RefPhi.msg.html

(cl:defclass <RefPhi> (roslisp-msg-protocol:ros-message)
  ((phi
    :reader phi
    :initarg :phi
    :type cl:float
    :initform 0.0)
   (dphi
    :reader dphi
    :initarg :dphi
    :type cl:float
    :initform 0.0))
)

(cl:defclass RefPhi (<RefPhi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RefPhi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RefPhi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<RefPhi> is deprecated: use magmed_msgs-msg:RefPhi instead.")))

(cl:ensure-generic-function 'phi-val :lambda-list '(m))
(cl:defmethod phi-val ((m <RefPhi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:phi-val is deprecated.  Use magmed_msgs-msg:phi instead.")
  (phi m))

(cl:ensure-generic-function 'dphi-val :lambda-list '(m))
(cl:defmethod dphi-val ((m <RefPhi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:dphi-val is deprecated.  Use magmed_msgs-msg:dphi instead.")
  (dphi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RefPhi>) ostream)
  "Serializes a message object of type '<RefPhi>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'phi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dphi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RefPhi>) istream)
  "Deserializes a message object of type '<RefPhi>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dphi) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RefPhi>)))
  "Returns string type for a message object of type '<RefPhi>"
  "magmed_msgs/RefPhi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RefPhi)))
  "Returns string type for a message object of type 'RefPhi"
  "magmed_msgs/RefPhi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RefPhi>)))
  "Returns md5sum for a message object of type '<RefPhi>"
  "55e8d7deb213bddd90abc0b338391bbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RefPhi)))
  "Returns md5sum for a message object of type 'RefPhi"
  "55e8d7deb213bddd90abc0b338391bbc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RefPhi>)))
  "Returns full string definition for message of type '<RefPhi>"
  (cl:format cl:nil "float64 phi~%float64 dphi~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RefPhi)))
  "Returns full string definition for message of type 'RefPhi"
  (cl:format cl:nil "float64 phi~%float64 dphi~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RefPhi>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RefPhi>))
  "Converts a ROS message object to a list"
  (cl:list 'RefPhi
    (cl:cons ':phi (phi msg))
    (cl:cons ':dphi (dphi msg))
))

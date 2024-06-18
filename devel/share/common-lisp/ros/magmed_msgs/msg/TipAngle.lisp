; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude TipAngle.msg.html

(cl:defclass <TipAngle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tipAngle
    :reader tipAngle
    :initarg :tipAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass TipAngle (<TipAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TipAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TipAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<TipAngle> is deprecated: use magmed_msgs-msg:TipAngle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TipAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:header-val is deprecated.  Use magmed_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tipAngle-val :lambda-list '(m))
(cl:defmethod tipAngle-val ((m <TipAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:tipAngle-val is deprecated.  Use magmed_msgs-msg:tipAngle instead.")
  (tipAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TipAngle>) ostream)
  "Serializes a message object of type '<TipAngle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tipAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TipAngle>) istream)
  "Deserializes a message object of type '<TipAngle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tipAngle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TipAngle>)))
  "Returns string type for a message object of type '<TipAngle>"
  "magmed_msgs/TipAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TipAngle)))
  "Returns string type for a message object of type 'TipAngle"
  "magmed_msgs/TipAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TipAngle>)))
  "Returns md5sum for a message object of type '<TipAngle>"
  "53e2c53db502ae3cd8c298da558777c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TipAngle)))
  "Returns md5sum for a message object of type 'TipAngle"
  "53e2c53db502ae3cd8c298da558777c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TipAngle>)))
  "Returns full string definition for message of type '<TipAngle>"
  (cl:format cl:nil "Header header~%float64 tipAngle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TipAngle)))
  "Returns full string definition for message of type 'TipAngle"
  (cl:format cl:nil "Header header~%float64 tipAngle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TipAngle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TipAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'TipAngle
    (cl:cons ':header (header msg))
    (cl:cons ':tipAngle (tipAngle msg))
))

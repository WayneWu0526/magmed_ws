; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude RoboStates.msg.html

(cl:defclass <RoboStates> (roslisp-msg-protocol:ros-message)
  ((VAL
    :reader VAL
    :initarg :VAL
    :type cl:integer
    :initform 0))
)

(cl:defclass RoboStates (<RoboStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<RoboStates> is deprecated: use magmed_msgs-msg:RoboStates instead.")))

(cl:ensure-generic-function 'VAL-val :lambda-list '(m))
(cl:defmethod VAL-val ((m <RoboStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:VAL-val is deprecated.  Use magmed_msgs-msg:VAL instead.")
  (VAL m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RoboStates>)))
    "Constants for message type '<RoboStates>"
  '((:INIT . 0)
    (:RUN . 1)
    (:TERM . -1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RoboStates)))
    "Constants for message type 'RoboStates"
  '((:INIT . 0)
    (:RUN . 1)
    (:TERM . -1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboStates>) ostream)
  "Serializes a message object of type '<RoboStates>"
  (cl:let* ((signed (cl:slot-value msg 'VAL)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboStates>) istream)
  "Deserializes a message object of type '<RoboStates>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'VAL) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoboStates>)))
  "Returns string type for a message object of type '<RoboStates>"
  "magmed_msgs/RoboStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboStates)))
  "Returns string type for a message object of type 'RoboStates"
  "magmed_msgs/RoboStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoboStates>)))
  "Returns md5sum for a message object of type '<RoboStates>"
  "3f7db48bba6b67d991a886a8d887cb31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboStates)))
  "Returns md5sum for a message object of type 'RoboStates"
  "3f7db48bba6b67d991a886a8d887cb31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboStates>)))
  "Returns full string definition for message of type '<RoboStates>"
  (cl:format cl:nil "int32  INIT = 0    # 初始化状态~%int32  RUN = 1  # 运行状态~%int32  TERM = -1    # 终止状态~%int32 VAL~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboStates)))
  "Returns full string definition for message of type 'RoboStates"
  (cl:format cl:nil "int32  INIT = 0    # 初始化状态~%int32  RUN = 1  # 运行状态~%int32  TERM = -1    # 终止状态~%int32 VAL~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboStates>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboStates>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboStates
    (cl:cons ':VAL (VAL msg))
))

; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude RoboStates.msg.html

(cl:defclass <RoboStates> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RoboStates (<RoboStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<RoboStates> is deprecated: use magmed_msgs-msg:RoboStates instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RoboStates>)))
    "Constants for message type '<RoboStates>"
  '((:INIT . 0)
    (:CTRL . 1)
    (:TERM . -1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RoboStates)))
    "Constants for message type 'RoboStates"
  '((:INIT . 0)
    (:CTRL . 1)
    (:TERM . -1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboStates>) ostream)
  "Serializes a message object of type '<RoboStates>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboStates>) istream)
  "Deserializes a message object of type '<RoboStates>"
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
  "9f9deeb86e59ce718cd341cb3f03488c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboStates)))
  "Returns md5sum for a message object of type 'RoboStates"
  "9f9deeb86e59ce718cd341cb3f03488c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboStates>)))
  "Returns full string definition for message of type '<RoboStates>"
  (cl:format cl:nil "int8  INIT = 0    # 初始化状态~%int8  CTRL = 1    # 控制状态~%int8  TERM = -1    # 终止状态~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboStates)))
  "Returns full string definition for message of type 'RoboStates"
  (cl:format cl:nil "int8  INIT = 0    # 初始化状态~%int8  CTRL = 1    # 控制状态~%int8  TERM = -1    # 终止状态~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboStates>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboStates>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboStates
))

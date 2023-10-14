; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude RoboStates.msg.html

(cl:defclass <RoboStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass RoboStates (<RoboStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<RoboStates> is deprecated: use magmed_msgs-msg:RoboStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RoboStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:header-val is deprecated.  Use magmed_msgs-msg:header instead.")
  (header m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RoboStates>)))
    "Constants for message type '<RoboStates>"
  '((:INIT . 0)
    (:CTRL . 1)
    (:TERM . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RoboStates)))
    "Constants for message type 'RoboStates"
  '((:INIT . 0)
    (:CTRL . 1)
    (:TERM . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboStates>) ostream)
  "Serializes a message object of type '<RoboStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboStates>) istream)
  "Deserializes a message object of type '<RoboStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
  "4437f8da4929f4c392a982caafd44e62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboStates)))
  "Returns md5sum for a message object of type 'RoboStates"
  "4437f8da4929f4c392a982caafd44e62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboStates>)))
  "Returns full string definition for message of type '<RoboStates>"
  (cl:format cl:nil "Header header~%int8  INIT = 0    # 初始化状态~%int8  CTRL = 1    # 控制状态~%int8  TERM = 2    # 终止状态~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboStates)))
  "Returns full string definition for message of type 'RoboStates"
  (cl:format cl:nil "Header header~%int8  INIT = 0    # 初始化状态~%int8  CTRL = 1    # 控制状态~%int8  TERM = 2    # 终止状态~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboStates>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboStates
    (cl:cons ':header (header msg))
))

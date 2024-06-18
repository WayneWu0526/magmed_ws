; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-srv)


;//! \htmlinclude SelfCollisionCheck-request.msg.html

(cl:defclass <SelfCollisionCheck-request> (roslisp-msg-protocol:ros-message)
  ((joints
    :reader joints
    :initarg :joints
    :type magmed_msgs-msg:RoboJoints
    :initform (cl:make-instance 'magmed_msgs-msg:RoboJoints)))
)

(cl:defclass SelfCollisionCheck-request (<SelfCollisionCheck-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelfCollisionCheck-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelfCollisionCheck-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-srv:<SelfCollisionCheck-request> is deprecated: use magmed_msgs-srv:SelfCollisionCheck-request instead.")))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <SelfCollisionCheck-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-srv:joints-val is deprecated.  Use magmed_msgs-srv:joints instead.")
  (joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelfCollisionCheck-request>) ostream)
  "Serializes a message object of type '<SelfCollisionCheck-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joints) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelfCollisionCheck-request>) istream)
  "Deserializes a message object of type '<SelfCollisionCheck-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joints) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelfCollisionCheck-request>)))
  "Returns string type for a service object of type '<SelfCollisionCheck-request>"
  "magmed_msgs/SelfCollisionCheckRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelfCollisionCheck-request)))
  "Returns string type for a service object of type 'SelfCollisionCheck-request"
  "magmed_msgs/SelfCollisionCheckRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelfCollisionCheck-request>)))
  "Returns md5sum for a message object of type '<SelfCollisionCheck-request>"
  "7060f063846e94f79170ebac9812a07a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelfCollisionCheck-request)))
  "Returns md5sum for a message object of type 'SelfCollisionCheck-request"
  "7060f063846e94f79170ebac9812a07a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelfCollisionCheck-request>)))
  "Returns full string definition for message of type '<SelfCollisionCheck-request>"
  (cl:format cl:nil "RoboJoints joints~%~%================================================================================~%MSG: magmed_msgs/RoboJoints~%Header header~%float64[] joints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelfCollisionCheck-request)))
  "Returns full string definition for message of type 'SelfCollisionCheck-request"
  (cl:format cl:nil "RoboJoints joints~%~%================================================================================~%MSG: magmed_msgs/RoboJoints~%Header header~%float64[] joints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelfCollisionCheck-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joints))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelfCollisionCheck-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelfCollisionCheck-request
    (cl:cons ':joints (joints msg))
))
;//! \htmlinclude SelfCollisionCheck-response.msg.html

(cl:defclass <SelfCollisionCheck-response> (roslisp-msg-protocol:ros-message)
  ((checkResult
    :reader checkResult
    :initarg :checkResult
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SelfCollisionCheck-response (<SelfCollisionCheck-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelfCollisionCheck-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelfCollisionCheck-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-srv:<SelfCollisionCheck-response> is deprecated: use magmed_msgs-srv:SelfCollisionCheck-response instead.")))

(cl:ensure-generic-function 'checkResult-val :lambda-list '(m))
(cl:defmethod checkResult-val ((m <SelfCollisionCheck-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-srv:checkResult-val is deprecated.  Use magmed_msgs-srv:checkResult instead.")
  (checkResult m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelfCollisionCheck-response>) ostream)
  "Serializes a message object of type '<SelfCollisionCheck-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'checkResult) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelfCollisionCheck-response>) istream)
  "Deserializes a message object of type '<SelfCollisionCheck-response>"
    (cl:setf (cl:slot-value msg 'checkResult) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelfCollisionCheck-response>)))
  "Returns string type for a service object of type '<SelfCollisionCheck-response>"
  "magmed_msgs/SelfCollisionCheckResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelfCollisionCheck-response)))
  "Returns string type for a service object of type 'SelfCollisionCheck-response"
  "magmed_msgs/SelfCollisionCheckResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelfCollisionCheck-response>)))
  "Returns md5sum for a message object of type '<SelfCollisionCheck-response>"
  "7060f063846e94f79170ebac9812a07a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelfCollisionCheck-response)))
  "Returns md5sum for a message object of type 'SelfCollisionCheck-response"
  "7060f063846e94f79170ebac9812a07a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelfCollisionCheck-response>)))
  "Returns full string definition for message of type '<SelfCollisionCheck-response>"
  (cl:format cl:nil "bool checkResult~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelfCollisionCheck-response)))
  "Returns full string definition for message of type 'SelfCollisionCheck-response"
  (cl:format cl:nil "bool checkResult~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelfCollisionCheck-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelfCollisionCheck-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelfCollisionCheck-response
    (cl:cons ':checkResult (checkResult msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelfCollisionCheck)))
  'SelfCollisionCheck-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelfCollisionCheck)))
  'SelfCollisionCheck-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelfCollisionCheck)))
  "Returns string type for a service object of type '<SelfCollisionCheck>"
  "magmed_msgs/SelfCollisionCheck")
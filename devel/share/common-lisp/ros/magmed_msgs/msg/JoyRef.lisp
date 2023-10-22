; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude JoyRef.msg.html

(cl:defclass <JoyRef> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (refPhi
    :reader refPhi
    :initarg :refPhi
    :type magmed_msgs-msg:RefPhi
    :initform (cl:make-instance 'magmed_msgs-msg:RefPhi))
   (refTheta
    :reader refTheta
    :initarg :refTheta
    :type magmed_msgs-msg:RefTheta
    :initform (cl:make-instance 'magmed_msgs-msg:RefTheta)))
)

(cl:defclass JoyRef (<JoyRef>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyRef>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyRef)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<JoyRef> is deprecated: use magmed_msgs-msg:JoyRef instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JoyRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:header-val is deprecated.  Use magmed_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'refPhi-val :lambda-list '(m))
(cl:defmethod refPhi-val ((m <JoyRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:refPhi-val is deprecated.  Use magmed_msgs-msg:refPhi instead.")
  (refPhi m))

(cl:ensure-generic-function 'refTheta-val :lambda-list '(m))
(cl:defmethod refTheta-val ((m <JoyRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:refTheta-val is deprecated.  Use magmed_msgs-msg:refTheta instead.")
  (refTheta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyRef>) ostream)
  "Serializes a message object of type '<JoyRef>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'refPhi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'refTheta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyRef>) istream)
  "Deserializes a message object of type '<JoyRef>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'refPhi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'refTheta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyRef>)))
  "Returns string type for a message object of type '<JoyRef>"
  "magmed_msgs/JoyRef")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyRef)))
  "Returns string type for a message object of type 'JoyRef"
  "magmed_msgs/JoyRef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyRef>)))
  "Returns md5sum for a message object of type '<JoyRef>"
  "ea53e564f4388a7ea7a788d618611b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyRef)))
  "Returns md5sum for a message object of type 'JoyRef"
  "ea53e564f4388a7ea7a788d618611b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyRef>)))
  "Returns full string definition for message of type '<JoyRef>"
  (cl:format cl:nil "Header header~%RefPhi refPhi ~%RefTheta refTheta~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: magmed_msgs/RefPhi~%float64 phi~%float64 dphi~%================================================================================~%MSG: magmed_msgs/RefTheta~%float64 theta~%float64 dtheta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyRef)))
  "Returns full string definition for message of type 'JoyRef"
  (cl:format cl:nil "Header header~%RefPhi refPhi ~%RefTheta refTheta~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: magmed_msgs/RefPhi~%float64 phi~%float64 dphi~%================================================================================~%MSG: magmed_msgs/RefTheta~%float64 theta~%float64 dtheta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyRef>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'refPhi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'refTheta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyRef>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyRef
    (cl:cons ':header (header msg))
    (cl:cons ':refPhi (refPhi msg))
    (cl:cons ':refTheta (refTheta msg))
))

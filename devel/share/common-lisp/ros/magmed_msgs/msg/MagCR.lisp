; Auto-generated. Do not edit!


(cl:in-package magmed_msgs-msg)


;//! \htmlinclude MagCR.msg.html

(cl:defclass <MagCR> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (phi_mock
    :reader phi_mock
    :initarg :phi_mock
    :type cl:float
    :initform 0.0)
   (thetaL_mock
    :reader thetaL_mock
    :initarg :thetaL_mock
    :type cl:float
    :initform 0.0)
   (phi_msr
    :reader phi_msr
    :initarg :phi_msr
    :type cl:float
    :initform 0.0)
   (thetaL_msr
    :reader thetaL_msr
    :initarg :thetaL_msr
    :type cl:float
    :initform 0.0)
   (tipPoint
    :reader tipPoint
    :initarg :tipPoint
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (Tsg
    :reader Tsg
    :initarg :Tsg
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass MagCR (<MagCR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MagCR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MagCR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magmed_msgs-msg:<MagCR> is deprecated: use magmed_msgs-msg:MagCR instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:header-val is deprecated.  Use magmed_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'phi_mock-val :lambda-list '(m))
(cl:defmethod phi_mock-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:phi_mock-val is deprecated.  Use magmed_msgs-msg:phi_mock instead.")
  (phi_mock m))

(cl:ensure-generic-function 'thetaL_mock-val :lambda-list '(m))
(cl:defmethod thetaL_mock-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:thetaL_mock-val is deprecated.  Use magmed_msgs-msg:thetaL_mock instead.")
  (thetaL_mock m))

(cl:ensure-generic-function 'phi_msr-val :lambda-list '(m))
(cl:defmethod phi_msr-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:phi_msr-val is deprecated.  Use magmed_msgs-msg:phi_msr instead.")
  (phi_msr m))

(cl:ensure-generic-function 'thetaL_msr-val :lambda-list '(m))
(cl:defmethod thetaL_msr-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:thetaL_msr-val is deprecated.  Use magmed_msgs-msg:thetaL_msr instead.")
  (thetaL_msr m))

(cl:ensure-generic-function 'tipPoint-val :lambda-list '(m))
(cl:defmethod tipPoint-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:tipPoint-val is deprecated.  Use magmed_msgs-msg:tipPoint instead.")
  (tipPoint m))

(cl:ensure-generic-function 'Tsg-val :lambda-list '(m))
(cl:defmethod Tsg-val ((m <MagCR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magmed_msgs-msg:Tsg-val is deprecated.  Use magmed_msgs-msg:Tsg instead.")
  (Tsg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MagCR>) ostream)
  "Serializes a message object of type '<MagCR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'phi_mock))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thetaL_mock))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'phi_msr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thetaL_msr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tipPoint) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Tsg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MagCR>) istream)
  "Deserializes a message object of type '<MagCR>"
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
    (cl:setf (cl:slot-value msg 'phi_mock) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaL_mock) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi_msr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaL_msr) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tipPoint) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Tsg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MagCR>)))
  "Returns string type for a message object of type '<MagCR>"
  "magmed_msgs/MagCR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MagCR)))
  "Returns string type for a message object of type 'MagCR"
  "magmed_msgs/MagCR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MagCR>)))
  "Returns md5sum for a message object of type '<MagCR>"
  "d2b6af8e88cbe5faeaf2769acfe589ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MagCR)))
  "Returns md5sum for a message object of type 'MagCR"
  "d2b6af8e88cbe5faeaf2769acfe589ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MagCR>)))
  "Returns full string definition for message of type '<MagCR>"
  (cl:format cl:nil "Header header~%~%float64 phi_mock~%float64 thetaL_mock~%float64 phi_msr~%float64 thetaL_msr~%geometry_msgs/Point tipPoint~%geometry_msgs/Pose Tsg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MagCR)))
  "Returns full string definition for message of type 'MagCR"
  (cl:format cl:nil "Header header~%~%float64 phi_mock~%float64 thetaL_mock~%float64 phi_msr~%float64 thetaL_msr~%geometry_msgs/Point tipPoint~%geometry_msgs/Pose Tsg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MagCR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tipPoint))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Tsg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MagCR>))
  "Converts a ROS message object to a list"
  (cl:list 'MagCR
    (cl:cons ':header (header msg))
    (cl:cons ':phi_mock (phi_mock msg))
    (cl:cons ':thetaL_mock (thetaL_mock msg))
    (cl:cons ':phi_msr (phi_msr msg))
    (cl:cons ':thetaL_msr (thetaL_msr msg))
    (cl:cons ':tipPoint (tipPoint msg))
    (cl:cons ':Tsg (Tsg msg))
))

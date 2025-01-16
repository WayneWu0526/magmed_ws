// Auto-generated. Do not edit!

// (in-package magmed_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class MagCR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.phi_mock = null;
      this.thetaL_mock = null;
      this.phi_msr = null;
      this.thetaL_msr = null;
      this.tipPoint = null;
      this.Tsg = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('phi_mock')) {
        this.phi_mock = initObj.phi_mock
      }
      else {
        this.phi_mock = 0.0;
      }
      if (initObj.hasOwnProperty('thetaL_mock')) {
        this.thetaL_mock = initObj.thetaL_mock
      }
      else {
        this.thetaL_mock = 0.0;
      }
      if (initObj.hasOwnProperty('phi_msr')) {
        this.phi_msr = initObj.phi_msr
      }
      else {
        this.phi_msr = 0.0;
      }
      if (initObj.hasOwnProperty('thetaL_msr')) {
        this.thetaL_msr = initObj.thetaL_msr
      }
      else {
        this.thetaL_msr = 0.0;
      }
      if (initObj.hasOwnProperty('tipPoint')) {
        this.tipPoint = initObj.tipPoint
      }
      else {
        this.tipPoint = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('Tsg')) {
        this.Tsg = initObj.Tsg
      }
      else {
        this.Tsg = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MagCR
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [phi_mock]
    bufferOffset = _serializer.float64(obj.phi_mock, buffer, bufferOffset);
    // Serialize message field [thetaL_mock]
    bufferOffset = _serializer.float64(obj.thetaL_mock, buffer, bufferOffset);
    // Serialize message field [phi_msr]
    bufferOffset = _serializer.float64(obj.phi_msr, buffer, bufferOffset);
    // Serialize message field [thetaL_msr]
    bufferOffset = _serializer.float64(obj.thetaL_msr, buffer, bufferOffset);
    // Serialize message field [tipPoint]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.tipPoint, buffer, bufferOffset);
    // Serialize message field [Tsg]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.Tsg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MagCR
    let len;
    let data = new MagCR(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [phi_mock]
    data.phi_mock = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thetaL_mock]
    data.thetaL_mock = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [phi_msr]
    data.phi_msr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thetaL_msr]
    data.thetaL_msr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tipPoint]
    data.tipPoint = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [Tsg]
    data.Tsg = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/MagCR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2b6af8e88cbe5faeaf2769acfe589ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float64 phi_mock
    float64 thetaL_mock
    float64 phi_msr
    float64 thetaL_msr
    geometry_msgs/Point tipPoint
    geometry_msgs/Pose Tsg
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MagCR(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.phi_mock !== undefined) {
      resolved.phi_mock = msg.phi_mock;
    }
    else {
      resolved.phi_mock = 0.0
    }

    if (msg.thetaL_mock !== undefined) {
      resolved.thetaL_mock = msg.thetaL_mock;
    }
    else {
      resolved.thetaL_mock = 0.0
    }

    if (msg.phi_msr !== undefined) {
      resolved.phi_msr = msg.phi_msr;
    }
    else {
      resolved.phi_msr = 0.0
    }

    if (msg.thetaL_msr !== undefined) {
      resolved.thetaL_msr = msg.thetaL_msr;
    }
    else {
      resolved.thetaL_msr = 0.0
    }

    if (msg.tipPoint !== undefined) {
      resolved.tipPoint = geometry_msgs.msg.Point.Resolve(msg.tipPoint)
    }
    else {
      resolved.tipPoint = new geometry_msgs.msg.Point()
    }

    if (msg.Tsg !== undefined) {
      resolved.Tsg = geometry_msgs.msg.Pose.Resolve(msg.Tsg)
    }
    else {
      resolved.Tsg = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = MagCR;

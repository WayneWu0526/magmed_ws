// Auto-generated. Do not edit!

// (in-package magmed_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RoboJoints = require('../msg/RoboJoints.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SelfCollisionCheckRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints = null;
    }
    else {
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new RoboJoints();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelfCollisionCheckRequest
    // Serialize message field [joints]
    bufferOffset = RoboJoints.serialize(obj.joints, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelfCollisionCheckRequest
    let len;
    let data = new SelfCollisionCheckRequest(null);
    // Deserialize message field [joints]
    data.joints = RoboJoints.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RoboJoints.getMessageSize(object.joints);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'magmed_msgs/SelfCollisionCheckRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbf85f6e82ce9a948d141e2b479a32b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RoboJoints joints
    
    ================================================================================
    MSG: magmed_msgs/RoboJoints
    Header header
    float64[] joints
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelfCollisionCheckRequest(null);
    if (msg.joints !== undefined) {
      resolved.joints = RoboJoints.Resolve(msg.joints)
    }
    else {
      resolved.joints = new RoboJoints()
    }

    return resolved;
    }
};

class SelfCollisionCheckResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.checkResult = null;
    }
    else {
      if (initObj.hasOwnProperty('checkResult')) {
        this.checkResult = initObj.checkResult
      }
      else {
        this.checkResult = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelfCollisionCheckResponse
    // Serialize message field [checkResult]
    bufferOffset = _serializer.bool(obj.checkResult, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelfCollisionCheckResponse
    let len;
    let data = new SelfCollisionCheckResponse(null);
    // Deserialize message field [checkResult]
    data.checkResult = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'magmed_msgs/SelfCollisionCheckResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3429e6e099a21a76bf78e62a4b36ad3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool checkResult
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelfCollisionCheckResponse(null);
    if (msg.checkResult !== undefined) {
      resolved.checkResult = msg.checkResult;
    }
    else {
      resolved.checkResult = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SelfCollisionCheckRequest,
  Response: SelfCollisionCheckResponse,
  md5sum() { return '7060f063846e94f79170ebac9812a07a'; },
  datatype() { return 'magmed_msgs/SelfCollisionCheck'; }
};

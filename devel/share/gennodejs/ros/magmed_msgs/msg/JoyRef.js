// Auto-generated. Do not edit!

// (in-package magmed_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RefPhi = require('./RefPhi.js');
let RefTheta = require('./RefTheta.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class JoyRef {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.refPhi = null;
      this.refTheta = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('refPhi')) {
        this.refPhi = initObj.refPhi
      }
      else {
        this.refPhi = new RefPhi();
      }
      if (initObj.hasOwnProperty('refTheta')) {
        this.refTheta = initObj.refTheta
      }
      else {
        this.refTheta = new RefTheta();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JoyRef
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [refPhi]
    bufferOffset = RefPhi.serialize(obj.refPhi, buffer, bufferOffset);
    // Serialize message field [refTheta]
    bufferOffset = RefTheta.serialize(obj.refTheta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JoyRef
    let len;
    let data = new JoyRef(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [refPhi]
    data.refPhi = RefPhi.deserialize(buffer, bufferOffset);
    // Deserialize message field [refTheta]
    data.refTheta = RefTheta.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/JoyRef';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ea53e564f4388a7ea7a788d618611b29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    RefPhi refPhi 
    RefTheta refTheta
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
    MSG: magmed_msgs/RefPhi
    float64 phi
    float64 dphi
    ================================================================================
    MSG: magmed_msgs/RefTheta
    float64 theta
    float64 dtheta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JoyRef(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.refPhi !== undefined) {
      resolved.refPhi = RefPhi.Resolve(msg.refPhi)
    }
    else {
      resolved.refPhi = new RefPhi()
    }

    if (msg.refTheta !== undefined) {
      resolved.refTheta = RefTheta.Resolve(msg.refTheta)
    }
    else {
      resolved.refTheta = new RefTheta()
    }

    return resolved;
    }
};

module.exports = JoyRef;

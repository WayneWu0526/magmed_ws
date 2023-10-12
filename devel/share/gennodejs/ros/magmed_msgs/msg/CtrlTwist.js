// Auto-generated. Do not edit!

// (in-package magmed_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CtrlTwist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.psi = null;
      this.p = null;
    }
    else {
      if (initObj.hasOwnProperty('psi')) {
        this.psi = initObj.psi
      }
      else {
        this.psi = 0.0;
      }
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CtrlTwist
    // Serialize message field [psi]
    bufferOffset = _serializer.float64(obj.psi, buffer, bufferOffset);
    // Check that the constant length array field [p] has the right length
    if (obj.p.length !== 3) {
      throw new Error('Unable to serialize array field p - length must be 3')
    }
    // Serialize message field [p]
    bufferOffset = _arraySerializer.float64(obj.p, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CtrlTwist
    let len;
    let data = new CtrlTwist(null);
    // Deserialize message field [psi]
    data.psi = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [p]
    data.p = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/CtrlTwist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76917cdec57096cc983b4e09ed694109';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 psi
    float64[3] p
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CtrlTwist(null);
    if (msg.psi !== undefined) {
      resolved.psi = msg.psi;
    }
    else {
      resolved.psi = 0.0
    }

    if (msg.p !== undefined) {
      resolved.p = msg.p;
    }
    else {
      resolved.p = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = CtrlTwist;

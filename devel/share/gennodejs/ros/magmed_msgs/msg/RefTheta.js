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

class RefTheta {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta = null;
      this.dtheta = null;
    }
    else {
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
      if (initObj.hasOwnProperty('dtheta')) {
        this.dtheta = initObj.dtheta
      }
      else {
        this.dtheta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RefTheta
    // Serialize message field [theta]
    bufferOffset = _serializer.float64(obj.theta, buffer, bufferOffset);
    // Serialize message field [dtheta]
    bufferOffset = _serializer.float64(obj.dtheta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RefTheta
    let len;
    let data = new RefTheta(null);
    // Deserialize message field [theta]
    data.theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dtheta]
    data.dtheta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/RefTheta';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b4963a90c97bbf7fdb1b9c9e9a7d1576';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 theta
    float64 dtheta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RefTheta(null);
    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    if (msg.dtheta !== undefined) {
      resolved.dtheta = msg.dtheta;
    }
    else {
      resolved.dtheta = 0.0
    }

    return resolved;
    }
};

module.exports = RefTheta;

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

class RefPhi {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.phi = null;
      this.dphi = null;
    }
    else {
      if (initObj.hasOwnProperty('phi')) {
        this.phi = initObj.phi
      }
      else {
        this.phi = 0.0;
      }
      if (initObj.hasOwnProperty('dphi')) {
        this.dphi = initObj.dphi
      }
      else {
        this.dphi = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RefPhi
    // Serialize message field [phi]
    bufferOffset = _serializer.float64(obj.phi, buffer, bufferOffset);
    // Serialize message field [dphi]
    bufferOffset = _serializer.float64(obj.dphi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RefPhi
    let len;
    let data = new RefPhi(null);
    // Deserialize message field [phi]
    data.phi = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dphi]
    data.dphi = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/RefPhi';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55e8d7deb213bddd90abc0b338391bbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 phi
    float64 dphi
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RefPhi(null);
    if (msg.phi !== undefined) {
      resolved.phi = msg.phi;
    }
    else {
      resolved.phi = 0.0
    }

    if (msg.dphi !== undefined) {
      resolved.dphi = msg.dphi;
    }
    else {
      resolved.dphi = 0.0
    }

    return resolved;
    }
};

module.exports = RefPhi;

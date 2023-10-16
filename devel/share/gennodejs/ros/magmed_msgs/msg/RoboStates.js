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

class RoboStates {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoboStates
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoboStates
    let len;
    let data = new RoboStates(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/RoboStates';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f9deeb86e59ce718cd341cb3f03488c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8  INIT = 0    # 初始化状态
    int8  CTRL = 1    # 控制状态
    int8  TERM = -1    # 终止状态
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoboStates(null);
    return resolved;
    }
};

// Constants for message
RoboStates.Constants = {
  INIT: 0,
  CTRL: 1,
  TERM: -1,
}

module.exports = RoboStates;

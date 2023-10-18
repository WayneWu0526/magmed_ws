// Auto-generated. Do not edit!

// (in-package magmed_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetRobotStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request = null;
    }
    else {
      if (initObj.hasOwnProperty('request')) {
        this.request = initObj.request
      }
      else {
        this.request = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotStateRequest
    // Serialize message field [request]
    bufferOffset = _serializer.int32(obj.request, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotStateRequest
    let len;
    let data = new GetRobotStateRequest(null);
    // Deserialize message field [request]
    data.request = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'magmed_msgs/GetRobotStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '650f0ccd41c8f8d53ada80be6ddde434';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # GetRobotState.srv
    int32 request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotStateRequest(null);
    if (msg.request !== undefined) {
      resolved.request = msg.request;
    }
    else {
      resolved.request = 0
    }

    return resolved;
    }
};

class GetRobotStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotStateResponse
    // Serialize message field [response]
    bufferOffset = _serializer.int32(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotStateResponse
    let len;
    let data = new GetRobotStateResponse(null);
    // Deserialize message field [response]
    data.response = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'magmed_msgs/GetRobotStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f45f68e2feefb1307598e828e260b7d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 response
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotStateResponse(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetRobotStateRequest,
  Response: GetRobotStateResponse,
  md5sum() { return '51edd9dfd50014fde2b589cbf77706aa'; },
  datatype() { return 'magmed_msgs/GetRobotState'; }
};

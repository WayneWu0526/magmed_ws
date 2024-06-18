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

//-----------------------------------------------------------

class PFjoystick {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.nJOY1 = null;
      this.nJOY2 = null;
      this.nJOY3 = null;
      this.bJOYD = null;
      this.POTA = null;
      this.POTB = null;
      this.BANA = null;
      this.BANB = null;
      this.ENCA = null;
      this.ENCB = null;
      this.TOG = null;
      this.BUT = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('nJOY1')) {
        this.nJOY1 = initObj.nJOY1
      }
      else {
        this.nJOY1 = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('nJOY2')) {
        this.nJOY2 = initObj.nJOY2
      }
      else {
        this.nJOY2 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('nJOY3')) {
        this.nJOY3 = initObj.nJOY3
      }
      else {
        this.nJOY3 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('bJOYD')) {
        this.bJOYD = initObj.bJOYD
      }
      else {
        this.bJOYD = false;
      }
      if (initObj.hasOwnProperty('POTA')) {
        this.POTA = initObj.POTA
      }
      else {
        this.POTA = 0;
      }
      if (initObj.hasOwnProperty('POTB')) {
        this.POTB = initObj.POTB
      }
      else {
        this.POTB = 0;
      }
      if (initObj.hasOwnProperty('BANA')) {
        this.BANA = initObj.BANA
      }
      else {
        this.BANA = 0;
      }
      if (initObj.hasOwnProperty('BANB')) {
        this.BANB = initObj.BANB
      }
      else {
        this.BANB = 0;
      }
      if (initObj.hasOwnProperty('ENCA')) {
        this.ENCA = initObj.ENCA
      }
      else {
        this.ENCA = 0;
      }
      if (initObj.hasOwnProperty('ENCB')) {
        this.ENCB = initObj.ENCB
      }
      else {
        this.ENCB = 0;
      }
      if (initObj.hasOwnProperty('TOG')) {
        this.TOG = initObj.TOG
      }
      else {
        this.TOG = new Array(5).fill(0);
      }
      if (initObj.hasOwnProperty('BUT')) {
        this.BUT = initObj.BUT
      }
      else {
        this.BUT = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PFjoystick
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [nJOY1] has the right length
    if (obj.nJOY1.length !== 3) {
      throw new Error('Unable to serialize array field nJOY1 - length must be 3')
    }
    // Serialize message field [nJOY1]
    bufferOffset = _arraySerializer.float32(obj.nJOY1, buffer, bufferOffset, 3);
    // Check that the constant length array field [nJOY2] has the right length
    if (obj.nJOY2.length !== 2) {
      throw new Error('Unable to serialize array field nJOY2 - length must be 2')
    }
    // Serialize message field [nJOY2]
    bufferOffset = _arraySerializer.float32(obj.nJOY2, buffer, bufferOffset, 2);
    // Check that the constant length array field [nJOY3] has the right length
    if (obj.nJOY3.length !== 2) {
      throw new Error('Unable to serialize array field nJOY3 - length must be 2')
    }
    // Serialize message field [nJOY3]
    bufferOffset = _arraySerializer.float32(obj.nJOY3, buffer, bufferOffset, 2);
    // Serialize message field [bJOYD]
    bufferOffset = _serializer.bool(obj.bJOYD, buffer, bufferOffset);
    // Serialize message field [POTA]
    bufferOffset = _serializer.uint16(obj.POTA, buffer, bufferOffset);
    // Serialize message field [POTB]
    bufferOffset = _serializer.uint16(obj.POTB, buffer, bufferOffset);
    // Serialize message field [BANA]
    bufferOffset = _serializer.int32(obj.BANA, buffer, bufferOffset);
    // Serialize message field [BANB]
    bufferOffset = _serializer.int32(obj.BANB, buffer, bufferOffset);
    // Serialize message field [ENCA]
    bufferOffset = _serializer.int16(obj.ENCA, buffer, bufferOffset);
    // Serialize message field [ENCB]
    bufferOffset = _serializer.int16(obj.ENCB, buffer, bufferOffset);
    // Check that the constant length array field [TOG] has the right length
    if (obj.TOG.length !== 5) {
      throw new Error('Unable to serialize array field TOG - length must be 5')
    }
    // Serialize message field [TOG]
    bufferOffset = _arraySerializer.bool(obj.TOG, buffer, bufferOffset, 5);
    // Check that the constant length array field [BUT] has the right length
    if (obj.BUT.length !== 6) {
      throw new Error('Unable to serialize array field BUT - length must be 6')
    }
    // Serialize message field [BUT]
    bufferOffset = _arraySerializer.bool(obj.BUT, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PFjoystick
    let len;
    let data = new PFjoystick(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [nJOY1]
    data.nJOY1 = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [nJOY2]
    data.nJOY2 = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [nJOY3]
    data.nJOY3 = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [bJOYD]
    data.bJOYD = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [POTA]
    data.POTA = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [POTB]
    data.POTB = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [BANA]
    data.BANA = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [BANB]
    data.BANB = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ENCA]
    data.ENCA = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ENCB]
    data.ENCB = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [TOG]
    data.TOG = _arrayDeserializer.bool(buffer, bufferOffset, 5)
    // Deserialize message field [BUT]
    data.BUT = _arrayDeserializer.bool(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'magmed_msgs/PFjoystick';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1b2838b51e4cc36d6636da93093d28d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header              # ROS standard header
    
    float32[3] nJOY1             # Three axes of the big joystick
    float32[2] nJOY2             # Two axes of the first (left) small joystick
    float32[2] nJOY3             # Two axes of the second (right) small joystick
    bool bJOYD                 # Big joystick button
    uint16 POTA                # Potentiometer A
    uint16 POTB                # Potentiometer B
    int32 BANA                 # Rotary switch A
    int32 BANB                 # Rotary switch B
    int16 ENCA                 # Encoder A
    int16 ENCB                 # Encoder B
    bool[5] TOG                # Toggle switches (5 in total)
    bool[6] BUT                # Push buttons (6 in total)
    
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
    const resolved = new PFjoystick(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.nJOY1 !== undefined) {
      resolved.nJOY1 = msg.nJOY1;
    }
    else {
      resolved.nJOY1 = new Array(3).fill(0)
    }

    if (msg.nJOY2 !== undefined) {
      resolved.nJOY2 = msg.nJOY2;
    }
    else {
      resolved.nJOY2 = new Array(2).fill(0)
    }

    if (msg.nJOY3 !== undefined) {
      resolved.nJOY3 = msg.nJOY3;
    }
    else {
      resolved.nJOY3 = new Array(2).fill(0)
    }

    if (msg.bJOYD !== undefined) {
      resolved.bJOYD = msg.bJOYD;
    }
    else {
      resolved.bJOYD = false
    }

    if (msg.POTA !== undefined) {
      resolved.POTA = msg.POTA;
    }
    else {
      resolved.POTA = 0
    }

    if (msg.POTB !== undefined) {
      resolved.POTB = msg.POTB;
    }
    else {
      resolved.POTB = 0
    }

    if (msg.BANA !== undefined) {
      resolved.BANA = msg.BANA;
    }
    else {
      resolved.BANA = 0
    }

    if (msg.BANB !== undefined) {
      resolved.BANB = msg.BANB;
    }
    else {
      resolved.BANB = 0
    }

    if (msg.ENCA !== undefined) {
      resolved.ENCA = msg.ENCA;
    }
    else {
      resolved.ENCA = 0
    }

    if (msg.ENCB !== undefined) {
      resolved.ENCB = msg.ENCB;
    }
    else {
      resolved.ENCB = 0
    }

    if (msg.TOG !== undefined) {
      resolved.TOG = msg.TOG;
    }
    else {
      resolved.TOG = new Array(5).fill(0)
    }

    if (msg.BUT !== undefined) {
      resolved.BUT = msg.BUT;
    }
    else {
      resolved.BUT = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = PFjoystick;

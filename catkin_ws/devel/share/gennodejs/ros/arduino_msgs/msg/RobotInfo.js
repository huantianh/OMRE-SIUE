// Auto-generated. Do not edit!

// (in-package arduino_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RobotInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ultraSonic = null;
      this.ir = null;
      this.enconder = null;
    }
    else {
      if (initObj.hasOwnProperty('ultraSonic')) {
        this.ultraSonic = initObj.ultraSonic
      }
      else {
        this.ultraSonic = [];
      }
      if (initObj.hasOwnProperty('ir')) {
        this.ir = initObj.ir
      }
      else {
        this.ir = [];
      }
      if (initObj.hasOwnProperty('enconder')) {
        this.enconder = initObj.enconder
      }
      else {
        this.enconder = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotInfo
    // Serialize message field [ultraSonic]
    bufferOffset = _arraySerializer.float32(obj.ultraSonic, buffer, bufferOffset, null);
    // Serialize message field [ir]
    bufferOffset = _arraySerializer.float32(obj.ir, buffer, bufferOffset, null);
    // Serialize message field [enconder]
    bufferOffset = _arraySerializer.int32(obj.enconder, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotInfo
    let len;
    let data = new RobotInfo(null);
    // Deserialize message field [ultraSonic]
    data.ultraSonic = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [ir]
    data.ir = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [enconder]
    data.enconder = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.ultraSonic.length;
    length += 4 * object.ir.length;
    length += 4 * object.enconder.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arduino_msgs/RobotInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7a9cbc08a00a37b339035a0f6b3cd87f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] ultraSonic
    float32[] ir
    int32[] enconder
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotInfo(null);
    if (msg.ultraSonic !== undefined) {
      resolved.ultraSonic = msg.ultraSonic;
    }
    else {
      resolved.ultraSonic = []
    }

    if (msg.ir !== undefined) {
      resolved.ir = msg.ir;
    }
    else {
      resolved.ir = []
    }

    if (msg.enconder !== undefined) {
      resolved.enconder = msg.enconder;
    }
    else {
      resolved.enconder = []
    }

    return resolved;
    }
};

module.exports = RobotInfo;

// Auto-generated. Do not edit!

// (in-package esp32_motor_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.steps = null;
      this.direction = null;
      this.enable = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('steps')) {
        this.steps = initObj.steps
      }
      else {
        this.steps = 0;
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = false;
      }
      if (initObj.hasOwnProperty('enable')) {
        this.enable = initObj.enable
      }
      else {
        this.enable = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCommand
    // Serialize message field [speed]
    bufferOffset = _serializer.int32(obj.speed, buffer, bufferOffset);
    // Serialize message field [steps]
    bufferOffset = _serializer.int32(obj.steps, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.bool(obj.direction, buffer, bufferOffset);
    // Serialize message field [enable]
    bufferOffset = _serializer.bool(obj.enable, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCommand
    let len;
    let data = new MotorCommand(null);
    // Deserialize message field [speed]
    data.speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [steps]
    data.steps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [enable]
    data.enable = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'esp32_motor_control/MotorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '940b5105a71761b8262aeb4d75bf0d20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 speed
    int32 steps
    bool direction
    bool enable
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCommand(null);
    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.steps !== undefined) {
      resolved.steps = msg.steps;
    }
    else {
      resolved.steps = 0
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = false
    }

    if (msg.enable !== undefined) {
      resolved.enable = msg.enable;
    }
    else {
      resolved.enable = false
    }

    return resolved;
    }
};

module.exports = MotorCommand;

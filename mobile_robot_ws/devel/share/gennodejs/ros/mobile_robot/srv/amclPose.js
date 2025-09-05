// Auto-generated. Do not edit!

// (in-package mobile_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class amclPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.getPose = null;
    }
    else {
      if (initObj.hasOwnProperty('getPose')) {
        this.getPose = initObj.getPose
      }
      else {
        this.getPose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type amclPoseRequest
    // Serialize message field [getPose]
    bufferOffset = _serializer.bool(obj.getPose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type amclPoseRequest
    let len;
    let data = new amclPoseRequest(null);
    // Deserialize message field [getPose]
    data.getPose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mobile_robot/amclPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9559bd3229841b2a55acb588ffb4beec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool getPose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new amclPoseRequest(null);
    if (msg.getPose !== undefined) {
      resolved.getPose = msg.getPose;
    }
    else {
      resolved.getPose = false
    }

    return resolved;
    }
};

class amclPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_x = null;
      this.pos_y = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_x')) {
        this.pos_x = initObj.pos_x
      }
      else {
        this.pos_x = 0.0;
      }
      if (initObj.hasOwnProperty('pos_y')) {
        this.pos_y = initObj.pos_y
      }
      else {
        this.pos_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type amclPoseResponse
    // Serialize message field [pos_x]
    bufferOffset = _serializer.float32(obj.pos_x, buffer, bufferOffset);
    // Serialize message field [pos_y]
    bufferOffset = _serializer.float32(obj.pos_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type amclPoseResponse
    let len;
    let data = new amclPoseResponse(null);
    // Deserialize message field [pos_x]
    data.pos_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pos_y]
    data.pos_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mobile_robot/amclPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2178591bb5b4f72502724261422523b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pos_x
    float32 pos_y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new amclPoseResponse(null);
    if (msg.pos_x !== undefined) {
      resolved.pos_x = msg.pos_x;
    }
    else {
      resolved.pos_x = 0.0
    }

    if (msg.pos_y !== undefined) {
      resolved.pos_y = msg.pos_y;
    }
    else {
      resolved.pos_y = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: amclPoseRequest,
  Response: amclPoseResponse,
  md5sum() { return '93e35a8b398245d681b8a701db81ea2e'; },
  datatype() { return 'mobile_robot/amclPose'; }
};

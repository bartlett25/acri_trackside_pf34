// Auto-generated. Do not edit!

// (in-package acri_localisation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let railLine = require('./railLine.js');

//-----------------------------------------------------------

class railPair {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.line1 = null;
      this.line2 = null;
      this.midline = null;
    }
    else {
      if (initObj.hasOwnProperty('line1')) {
        this.line1 = initObj.line1
      }
      else {
        this.line1 = new railLine();
      }
      if (initObj.hasOwnProperty('line2')) {
        this.line2 = initObj.line2
      }
      else {
        this.line2 = new railLine();
      }
      if (initObj.hasOwnProperty('midline')) {
        this.midline = initObj.midline
      }
      else {
        this.midline = new railLine();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type railPair
    // Serialize message field [line1]
    bufferOffset = railLine.serialize(obj.line1, buffer, bufferOffset);
    // Serialize message field [line2]
    bufferOffset = railLine.serialize(obj.line2, buffer, bufferOffset);
    // Serialize message field [midline]
    bufferOffset = railLine.serialize(obj.midline, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type railPair
    let len;
    let data = new railPair(null);
    // Deserialize message field [line1]
    data.line1 = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [line2]
    data.line2 = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [midline]
    data.midline = railLine.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 144;
  }

  static datatype() {
    // Returns string type for a message object
    return 'acri_localisation/railPair';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd47e372c1831bf7059fde9b5559b9f11';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    acri_localisation/railLine line1
    acri_localisation/railLine line2
    acri_localisation/railLine midline
    ================================================================================
    MSG: acri_localisation/railLine
    geometry_msgs/Point point1
    geometry_msgs/Point point2
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new railPair(null);
    if (msg.line1 !== undefined) {
      resolved.line1 = railLine.Resolve(msg.line1)
    }
    else {
      resolved.line1 = new railLine()
    }

    if (msg.line2 !== undefined) {
      resolved.line2 = railLine.Resolve(msg.line2)
    }
    else {
      resolved.line2 = new railLine()
    }

    if (msg.midline !== undefined) {
      resolved.midline = railLine.Resolve(msg.midline)
    }
    else {
      resolved.midline = new railLine()
    }

    return resolved;
    }
};

module.exports = railPair;

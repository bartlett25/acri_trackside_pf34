// Auto-generated. Do not edit!

// (in-package acri_localisation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class railLine {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point1 = null;
      this.point2 = null;
    }
    else {
      if (initObj.hasOwnProperty('point1')) {
        this.point1 = initObj.point1
      }
      else {
        this.point1 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('point2')) {
        this.point2 = initObj.point2
      }
      else {
        this.point2 = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type railLine
    // Serialize message field [point1]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.point1, buffer, bufferOffset);
    // Serialize message field [point2]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.point2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type railLine
    let len;
    let data = new railLine(null);
    // Deserialize message field [point1]
    data.point1 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [point2]
    data.point2 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'acri_localisation/railLine';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e377648347b19d625c7a86b684f82b75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new railLine(null);
    if (msg.point1 !== undefined) {
      resolved.point1 = geometry_msgs.msg.Point.Resolve(msg.point1)
    }
    else {
      resolved.point1 = new geometry_msgs.msg.Point()
    }

    if (msg.point2 !== undefined) {
      resolved.point2 = geometry_msgs.msg.Point.Resolve(msg.point2)
    }
    else {
      resolved.point2 = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = railLine;

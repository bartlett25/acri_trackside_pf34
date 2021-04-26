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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class railClosestPair {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.line1 = null;
      this.line2 = null;
      this.midline = null;
      this.inrange = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
      if (initObj.hasOwnProperty('inrange')) {
        this.inrange = initObj.inrange
      }
      else {
        this.inrange = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type railClosestPair
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [line1]
    bufferOffset = railLine.serialize(obj.line1, buffer, bufferOffset);
    // Serialize message field [line2]
    bufferOffset = railLine.serialize(obj.line2, buffer, bufferOffset);
    // Serialize message field [midline]
    bufferOffset = railLine.serialize(obj.midline, buffer, bufferOffset);
    // Serialize message field [inrange]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.inrange, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type railClosestPair
    let len;
    let data = new railClosestPair(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [line1]
    data.line1 = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [line2]
    data.line2 = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [midline]
    data.midline = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [inrange]
    data.inrange = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 145;
  }

  static datatype() {
    // Returns string type for a message object
    return 'acri_localisation/railClosestPair';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f8485efdc65afe9c5ad5817e1a262cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    acri_localisation/railLine line1
    acri_localisation/railLine line2
    acri_localisation/railLine midline
    std_msgs/Bool inrange
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
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new railClosestPair(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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

    if (msg.inrange !== undefined) {
      resolved.inrange = std_msgs.msg.Bool.Resolve(msg.inrange)
    }
    else {
      resolved.inrange = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = railClosestPair;

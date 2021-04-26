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
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class controlFromNUC {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.midline = null;
      this.pose2D = null;
      this.mode = null;
      this.inrange = null;
      this.voltage24 = null;
      this.voltage48 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('midline')) {
        this.midline = initObj.midline
      }
      else {
        this.midline = new railLine();
      }
      if (initObj.hasOwnProperty('pose2D')) {
        this.pose2D = initObj.pose2D
      }
      else {
        this.pose2D = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = new std_msgs.msg.UInt32();
      }
      if (initObj.hasOwnProperty('inrange')) {
        this.inrange = initObj.inrange
      }
      else {
        this.inrange = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('voltage24')) {
        this.voltage24 = initObj.voltage24
      }
      else {
        this.voltage24 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('voltage48')) {
        this.voltage48 = initObj.voltage48
      }
      else {
        this.voltage48 = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type controlFromNUC
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [midline]
    bufferOffset = railLine.serialize(obj.midline, buffer, bufferOffset);
    // Serialize message field [pose2D]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.pose2D, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = std_msgs.msg.UInt32.serialize(obj.mode, buffer, bufferOffset);
    // Serialize message field [inrange]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.inrange, buffer, bufferOffset);
    // Serialize message field [voltage24]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.voltage24, buffer, bufferOffset);
    // Serialize message field [voltage48]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.voltage48, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controlFromNUC
    let len;
    let data = new controlFromNUC(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [midline]
    data.midline = railLine.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose2D]
    data.pose2D = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = std_msgs.msg.UInt32.deserialize(buffer, bufferOffset);
    // Deserialize message field [inrange]
    data.inrange = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage24]
    data.voltage24 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage48]
    data.voltage48 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 85;
  }

  static datatype() {
    // Returns string type for a message object
    return 'acri_localisation/controlFromNUC';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd79800d54a13d23f4d7ba0216eb0f324';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # header: sequence and time-step id
    # mid line: rail mid_line comprising of two 3D points
    # pose2D: robot pose comprising of [x,y,theta] 
    # mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
    # inrange: rail is within valid range of vehicle to go into autonomous mode
    # voltage24: voltage of 24V battery
    # voltage48: voltage of 48V battery
    
    std_msgs/Header header
    acri_localisation/railLine midline
    geometry_msgs/Pose2D pose2D
    std_msgs/UInt32 mode
    std_msgs/Bool inrange
    std_msgs/Float32 voltage24
    std_msgs/Float32 voltage48
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
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    ================================================================================
    MSG: std_msgs/UInt32
    uint32 data
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new controlFromNUC(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.midline !== undefined) {
      resolved.midline = railLine.Resolve(msg.midline)
    }
    else {
      resolved.midline = new railLine()
    }

    if (msg.pose2D !== undefined) {
      resolved.pose2D = geometry_msgs.msg.Pose2D.Resolve(msg.pose2D)
    }
    else {
      resolved.pose2D = new geometry_msgs.msg.Pose2D()
    }

    if (msg.mode !== undefined) {
      resolved.mode = std_msgs.msg.UInt32.Resolve(msg.mode)
    }
    else {
      resolved.mode = new std_msgs.msg.UInt32()
    }

    if (msg.inrange !== undefined) {
      resolved.inrange = std_msgs.msg.Bool.Resolve(msg.inrange)
    }
    else {
      resolved.inrange = new std_msgs.msg.Bool()
    }

    if (msg.voltage24 !== undefined) {
      resolved.voltage24 = std_msgs.msg.Float32.Resolve(msg.voltage24)
    }
    else {
      resolved.voltage24 = new std_msgs.msg.Float32()
    }

    if (msg.voltage48 !== undefined) {
      resolved.voltage48 = std_msgs.msg.Float32.Resolve(msg.voltage48)
    }
    else {
      resolved.voltage48 = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

module.exports = controlFromNUC;

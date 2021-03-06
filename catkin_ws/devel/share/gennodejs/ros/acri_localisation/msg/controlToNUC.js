// Auto-generated. Do not edit!

// (in-package acri_localisation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class controlToNUC {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mode = null;
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
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = new std_msgs.msg.UInt32();
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
    // Serializes a message object of type controlToNUC
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = std_msgs.msg.UInt32.serialize(obj.mode, buffer, bufferOffset);
    // Serialize message field [voltage24]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.voltage24, buffer, bufferOffset);
    // Serialize message field [voltage48]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.voltage48, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controlToNUC
    let len;
    let data = new controlToNUC(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = std_msgs.msg.UInt32.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage24]
    data.voltage24 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage48]
    data.voltage48 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'acri_localisation/controlToNUC';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fae9323d5973f4783a5e2f3a2ec3199';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # header: sequence and time-step id
    # mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
    # voltage24: voltage of 24V battery
    # voltage48: voltage of 48V battery
    std_msgs/Header header
    std_msgs/UInt32 mode
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
    MSG: std_msgs/UInt32
    uint32 data
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
    const resolved = new controlToNUC(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mode !== undefined) {
      resolved.mode = std_msgs.msg.UInt32.Resolve(msg.mode)
    }
    else {
      resolved.mode = new std_msgs.msg.UInt32()
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

module.exports = controlToNUC;

// Auto-generated. Do not edit!

// (in-package custom_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class arm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.shoulder = null;
      this.elbow = null;
      this.wrist = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('shoulder')) {
        this.shoulder = initObj.shoulder
      }
      else {
        this.shoulder = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('elbow')) {
        this.elbow = initObj.elbow
      }
      else {
        this.elbow = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('wrist')) {
        this.wrist = initObj.wrist
      }
      else {
        this.wrist = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type arm
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [shoulder] has the right length
    if (obj.shoulder.length !== 3) {
      throw new Error('Unable to serialize array field shoulder - length must be 3')
    }
    // Serialize message field [shoulder]
    bufferOffset = _arraySerializer.float32(obj.shoulder, buffer, bufferOffset, 3);
    // Check that the constant length array field [elbow] has the right length
    if (obj.elbow.length !== 3) {
      throw new Error('Unable to serialize array field elbow - length must be 3')
    }
    // Serialize message field [elbow]
    bufferOffset = _arraySerializer.float32(obj.elbow, buffer, bufferOffset, 3);
    // Check that the constant length array field [wrist] has the right length
    if (obj.wrist.length !== 3) {
      throw new Error('Unable to serialize array field wrist - length must be 3')
    }
    // Serialize message field [wrist]
    bufferOffset = _arraySerializer.float32(obj.wrist, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type arm
    let len;
    let data = new arm(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [shoulder]
    data.shoulder = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [elbow]
    data.elbow = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [wrist]
    data.wrist = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/arm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f17acee9610e62286ac9052ebccb36dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32[3] shoulder
    float32[3] elbow
    float32[3] wrist
    
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
    const resolved = new arm(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.shoulder !== undefined) {
      resolved.shoulder = msg.shoulder;
    }
    else {
      resolved.shoulder = new Array(3).fill(0)
    }

    if (msg.elbow !== undefined) {
      resolved.elbow = msg.elbow;
    }
    else {
      resolved.elbow = new Array(3).fill(0)
    }

    if (msg.wrist !== undefined) {
      resolved.wrist = msg.wrist;
    }
    else {
      resolved.wrist = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = arm;

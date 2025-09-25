// Auto-generated. Do not edit!

// (in-package respeaker_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RawAudioData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.nb_channel = null;
      this.rate = null;
      this.format = null;
      this.sample_byte_size = null;
      this.nb_frames = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('nb_channel')) {
        this.nb_channel = initObj.nb_channel
      }
      else {
        this.nb_channel = 0;
      }
      if (initObj.hasOwnProperty('rate')) {
        this.rate = initObj.rate
      }
      else {
        this.rate = 0;
      }
      if (initObj.hasOwnProperty('format')) {
        this.format = initObj.format
      }
      else {
        this.format = 0;
      }
      if (initObj.hasOwnProperty('sample_byte_size')) {
        this.sample_byte_size = initObj.sample_byte_size
      }
      else {
        this.sample_byte_size = 0;
      }
      if (initObj.hasOwnProperty('nb_frames')) {
        this.nb_frames = initObj.nb_frames
      }
      else {
        this.nb_frames = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RawAudioData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [nb_channel]
    bufferOffset = _serializer.int8(obj.nb_channel, buffer, bufferOffset);
    // Serialize message field [rate]
    bufferOffset = _serializer.int32(obj.rate, buffer, bufferOffset);
    // Serialize message field [format]
    bufferOffset = _serializer.int32(obj.format, buffer, bufferOffset);
    // Serialize message field [sample_byte_size]
    bufferOffset = _serializer.int32(obj.sample_byte_size, buffer, bufferOffset);
    // Serialize message field [nb_frames]
    bufferOffset = _serializer.int32(obj.nb_frames, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int16(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RawAudioData
    let len;
    let data = new RawAudioData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [nb_channel]
    data.nb_channel = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [rate]
    data.rate = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [format]
    data.format = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sample_byte_size]
    data.sample_byte_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [nb_frames]
    data.nb_frames = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 2 * object.data.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'respeaker_ros/RawAudioData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c397debf40a72ecc26b4cfd85d2a668';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int8   nb_channel
    int32  rate
    int32  format
    int32  sample_byte_size
    int32  nb_frames
    int16[] data
    
    
    
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
    const resolved = new RawAudioData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.nb_channel !== undefined) {
      resolved.nb_channel = msg.nb_channel;
    }
    else {
      resolved.nb_channel = 0
    }

    if (msg.rate !== undefined) {
      resolved.rate = msg.rate;
    }
    else {
      resolved.rate = 0
    }

    if (msg.format !== undefined) {
      resolved.format = msg.format;
    }
    else {
      resolved.format = 0
    }

    if (msg.sample_byte_size !== undefined) {
      resolved.sample_byte_size = msg.sample_byte_size;
    }
    else {
      resolved.sample_byte_size = 0
    }

    if (msg.nb_frames !== undefined) {
      resolved.nb_frames = msg.nb_frames;
    }
    else {
      resolved.nb_frames = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = RawAudioData;

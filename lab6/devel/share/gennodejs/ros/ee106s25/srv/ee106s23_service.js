// Auto-generated. Do not edit!

// (in-package ee106s25.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ee106s23_serviceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('request_msg')) {
        this.request_msg = initObj.request_msg
      }
      else {
        this.request_msg = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ee106s23_serviceRequest
    // Serialize message field [request_msg]
    bufferOffset = _serializer.string(obj.request_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ee106s23_serviceRequest
    let len;
    let data = new ee106s23_serviceRequest(null);
    // Deserialize message field [request_msg]
    data.request_msg = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.request_msg);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ee106s25/ee106s23_serviceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec3985f3b00d27737504bd460abc4887';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string request_msg
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ee106s23_serviceRequest(null);
    if (msg.request_msg !== undefined) {
      resolved.request_msg = msg.request_msg;
    }
    else {
      resolved.request_msg = ''
    }

    return resolved;
    }
};

class ee106s23_serviceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('response_msg')) {
        this.response_msg = initObj.response_msg
      }
      else {
        this.response_msg = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ee106s23_serviceResponse
    // Serialize message field [response_msg]
    bufferOffset = _serializer.string(obj.response_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ee106s23_serviceResponse
    let len;
    let data = new ee106s23_serviceResponse(null);
    // Deserialize message field [response_msg]
    data.response_msg = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.response_msg);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ee106s25/ee106s23_serviceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7ff18183e9537ad289a116cf4aae4ef3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response_msg
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ee106s23_serviceResponse(null);
    if (msg.response_msg !== undefined) {
      resolved.response_msg = msg.response_msg;
    }
    else {
      resolved.response_msg = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ee106s23_serviceRequest,
  Response: ee106s23_serviceResponse,
  md5sum() { return 'bbbdbd457d086498a93ac52318048de7'; },
  datatype() { return 'ee106s25/ee106s23_service'; }
};

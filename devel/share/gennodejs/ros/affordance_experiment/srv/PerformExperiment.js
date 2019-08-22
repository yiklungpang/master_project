// Auto-generated. Do not edit!

// (in-package affordance_experiment.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PerformExperimentRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tool_name = null;
      this.object_list = null;
      this.repeat_no = null;
    }
    else {
      if (initObj.hasOwnProperty('tool_name')) {
        this.tool_name = initObj.tool_name
      }
      else {
        this.tool_name = '';
      }
      if (initObj.hasOwnProperty('object_list')) {
        this.object_list = initObj.object_list
      }
      else {
        this.object_list = [];
      }
      if (initObj.hasOwnProperty('repeat_no')) {
        this.repeat_no = initObj.repeat_no
      }
      else {
        this.repeat_no = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PerformExperimentRequest
    // Serialize message field [tool_name]
    bufferOffset = _serializer.string(obj.tool_name, buffer, bufferOffset);
    // Serialize message field [object_list]
    bufferOffset = _arraySerializer.string(obj.object_list, buffer, bufferOffset, null);
    // Serialize message field [repeat_no]
    bufferOffset = _serializer.int8(obj.repeat_no, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PerformExperimentRequest
    let len;
    let data = new PerformExperimentRequest(null);
    // Deserialize message field [tool_name]
    data.tool_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [object_list]
    data.object_list = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [repeat_no]
    data.repeat_no = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tool_name.length;
    object.object_list.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'affordance_experiment/PerformExperimentRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6fb0b1da9025d63f3831c18086cd1b40';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string tool_name
    string[] object_list
    int8 repeat_no
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PerformExperimentRequest(null);
    if (msg.tool_name !== undefined) {
      resolved.tool_name = msg.tool_name;
    }
    else {
      resolved.tool_name = ''
    }

    if (msg.object_list !== undefined) {
      resolved.object_list = msg.object_list;
    }
    else {
      resolved.object_list = []
    }

    if (msg.repeat_no !== undefined) {
      resolved.repeat_no = msg.repeat_no;
    }
    else {
      resolved.repeat_no = 0
    }

    return resolved;
    }
};

class PerformExperimentResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PerformExperimentResponse
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PerformExperimentResponse
    let len;
    let data = new PerformExperimentResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.status.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'affordance_experiment/PerformExperimentResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4fe5af303955c287688e7347e9b00278';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PerformExperimentResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: PerformExperimentRequest,
  Response: PerformExperimentResponse,
  md5sum() { return '6f5d2f09f803b7c0cfced3acb6c92077'; },
  datatype() { return 'affordance_experiment/PerformExperiment'; }
};

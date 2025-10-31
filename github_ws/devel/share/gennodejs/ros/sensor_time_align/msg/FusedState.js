// Auto-generated. Do not edit!

// (in-package sensor_time_align.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class FusedState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.image_filename = null;
      this.image = null;
      this.angular_velocity = null;
      this.linear_acceleration = null;
      this.imu_timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('image_filename')) {
        this.image_filename = initObj.image_filename
      }
      else {
        this.image_filename = '';
      }
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = [];
      }
      if (initObj.hasOwnProperty('linear_acceleration')) {
        this.linear_acceleration = initObj.linear_acceleration
      }
      else {
        this.linear_acceleration = [];
      }
      if (initObj.hasOwnProperty('imu_timestamp')) {
        this.imu_timestamp = initObj.imu_timestamp
      }
      else {
        this.imu_timestamp = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FusedState
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [image_filename]
    bufferOffset = _serializer.string(obj.image_filename, buffer, bufferOffset);
    // Serialize message field [image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.image, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    // Serialize the length for message field [angular_velocity]
    bufferOffset = _serializer.uint32(obj.angular_velocity.length, buffer, bufferOffset);
    obj.angular_velocity.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [linear_acceleration]
    // Serialize the length for message field [linear_acceleration]
    bufferOffset = _serializer.uint32(obj.linear_acceleration.length, buffer, bufferOffset);
    obj.linear_acceleration.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [imu_timestamp]
    bufferOffset = _arraySerializer.float64(obj.imu_timestamp, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FusedState
    let len;
    let data = new FusedState(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [image_filename]
    data.image_filename = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image]
    data.image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    // Deserialize array length for message field [angular_velocity]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.angular_velocity = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.angular_velocity[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [linear_acceleration]
    // Deserialize array length for message field [linear_acceleration]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.linear_acceleration = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.linear_acceleration[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [imu_timestamp]
    data.imu_timestamp = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.image_filename);
    length += sensor_msgs.msg.Image.getMessageSize(object.image);
    length += 24 * object.angular_velocity.length;
    length += 24 * object.linear_acceleration.length;
    length += 8 * object.imu_timestamp.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sensor_time_align/FusedState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc3086d10a35c265a9ccaae0f89428d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #对齐后的时间戳
    time timestamp
    #相机信息
    string image_filename                           #相机图片文件名
    sensor_msgs/Image image                         #图像数据
    #IMU 信息
    geometry_msgs/Vector3[] angular_velocity        # 存储多个 IMU 的角速度数据
    geometry_msgs/Vector3[] linear_acceleration     # 存储多个 IMU 的加速度数据
    # IMU 时间戳（每条 IMU 数据的时间戳）
    float64[] imu_timestamp  # 存储多个 IMU 数据的时间戳
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new FusedState(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.image_filename !== undefined) {
      resolved.image_filename = msg.image_filename;
    }
    else {
      resolved.image_filename = ''
    }

    if (msg.image !== undefined) {
      resolved.image = sensor_msgs.msg.Image.Resolve(msg.image)
    }
    else {
      resolved.image = new sensor_msgs.msg.Image()
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = new Array(msg.angular_velocity.length);
      for (let i = 0; i < resolved.angular_velocity.length; ++i) {
        resolved.angular_velocity[i] = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity[i]);
      }
    }
    else {
      resolved.angular_velocity = []
    }

    if (msg.linear_acceleration !== undefined) {
      resolved.linear_acceleration = new Array(msg.linear_acceleration.length);
      for (let i = 0; i < resolved.linear_acceleration.length; ++i) {
        resolved.linear_acceleration[i] = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration[i]);
      }
    }
    else {
      resolved.linear_acceleration = []
    }

    if (msg.imu_timestamp !== undefined) {
      resolved.imu_timestamp = msg.imu_timestamp;
    }
    else {
      resolved.imu_timestamp = []
    }

    return resolved;
    }
};

module.exports = FusedState;

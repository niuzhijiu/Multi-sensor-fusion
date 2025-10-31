;; Auto-generated. Do not edit!


(when (boundp 'sensor_time_align::FusedState)
  (if (not (find-package "SENSOR_TIME_ALIGN"))
    (make-package "SENSOR_TIME_ALIGN"))
  (shadow 'FusedState (find-package "SENSOR_TIME_ALIGN")))
(unless (find-package "SENSOR_TIME_ALIGN::FUSEDSTATE")
  (make-package "SENSOR_TIME_ALIGN::FUSEDSTATE"))

(in-package "ROS")
;;//! \htmlinclude FusedState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass sensor_time_align::FusedState
  :super ros::object
  :slots (_timestamp _image_filename _image _angular_velocity _linear_acceleration _imu_timestamp ))

(defmethod sensor_time_align::FusedState
  (:init
   (&key
    ((:timestamp __timestamp) (instance ros::time :init))
    ((:image_filename __image_filename) "")
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:angular_velocity __angular_velocity) ())
    ((:linear_acceleration __linear_acceleration) ())
    ((:imu_timestamp __imu_timestamp) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _timestamp __timestamp)
   (setq _image_filename (string __image_filename))
   (setq _image __image)
   (setq _angular_velocity __angular_velocity)
   (setq _linear_acceleration __linear_acceleration)
   (setq _imu_timestamp __imu_timestamp)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:image_filename
   (&optional __image_filename)
   (if __image_filename (setq _image_filename __image_filename)) _image_filename)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:imu_timestamp
   (&optional __imu_timestamp)
   (if __imu_timestamp (setq _imu_timestamp __imu_timestamp)) _imu_timestamp)
  (:serialization-length
   ()
   (+
    ;; time _timestamp
    8
    ;; string _image_filename
    4 (length _image_filename)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; geometry_msgs/Vector3[] _angular_velocity
    (apply #'+ (send-all _angular_velocity :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _linear_acceleration
    (apply #'+ (send-all _linear_acceleration :serialization-length)) 4
    ;; float64[] _imu_timestamp
    (* 8    (length _imu_timestamp)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;; string _image_filename
       (write-long (length _image_filename) s) (princ _image_filename s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; geometry_msgs/Vector3[] _angular_velocity
     (write-long (length _angular_velocity) s)
     (dolist (elem _angular_velocity)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _linear_acceleration
     (write-long (length _linear_acceleration) s)
     (dolist (elem _linear_acceleration)
       (send elem :serialize s)
       )
     ;; float64[] _imu_timestamp
     (write-long (length _imu_timestamp) s)
     (dotimes (i (length _imu_timestamp))
       (sys::poke (elt _imu_timestamp i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _image_filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; geometry_msgs/Vector3[] _angular_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _angular_velocity (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _angular_velocity)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _linear_acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _linear_acceleration (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _linear_acceleration)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _imu_timestamp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _imu_timestamp (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _imu_timestamp i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get sensor_time_align::FusedState :md5sum-) "dc3086d10a35c265a9ccaae0f89428d8")
(setf (get sensor_time_align::FusedState :datatype-) "sensor_time_align/FusedState")
(setf (get sensor_time_align::FusedState :definition-)
      "#对齐后的时间戳
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
")



(provide :sensor_time_align/FusedState "dc3086d10a35c265a9ccaae0f89428d8")



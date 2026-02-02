# multi-sensor_fusion
多传感融合项目，内有sensor_time_align、lidar_imu_calib、bw_ros_driver、rplidar_ros-master、livox_ros_driver2共5个功能包，能够实现2D激光雷达、3D激光雷达、六轴IMU、九轴IMU、普通单目相机、双目深度相机的驱动以及任意组合下的时间对齐和空间标定。

## 各功能包简介

### sensor_time_align
一个多传感器时空联合标定系统，由时间对齐部分和空间标定部分组成,此外还包括一些硬件驱动节点和数据集文件分割节点，能够实现雷达、IMU、相机的时间对齐和空间标定，为多传感器数据融合后具体场景的应用算法提供更加精确的数据。

### lidar_imu_calib
一个轻量级的雷达惯性标定工具，专为2D激光雷达与IMU的外参标定设计，核心包含数据采集、时间同步与外参求解三部分，无需依赖复杂的第三方标定工具，可直接在ROS1 Noetic环境下实时运行，适配LubanCat4等ARM架构硬件，能够高效、便捷地完成2D激光雷达与IMU的外参标定。

### bw_ros_driver
九轴IMU北微传感DMC620的官方驱动。

### rplidar_ros-master
2D激光雷达思岚rplidar的官方驱动。

### livox_ros_driver2
3D激光雷达览沃mid360的官方驱动。

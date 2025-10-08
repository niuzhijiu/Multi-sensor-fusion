# Multi-sensor-fusion
# sensor_time_align
基于ROS1 Noetic版的单目相机与IMU时间同步与数据融合工具

---

# 功能 Features
- 视觉角速度及角速度模长计算、imu角速度模长计算、时间偏移估计、时间戳校正
- 从CSV和图像目录读取相机与IMU数据并发布 `/cam_image` 与 `/imu_data`
- 将相机与区间IMU融合为自定义消息 `/fused_topic`并发布

---

# 环境 Environment
- Ubuntu 20.04 
- ROS1 Noetic 
- OpenCV 4.2.0
- C++11

# 使用 Usage
- 准备数据：相机CSV、图像、IMU CSV
- 编译运行cam_w_norm_calculate.cpp和imu_w_norm_calculate.cpp得到视觉角速度及角速度模长、imu角速度模长
- 编译运行time_offset_calculate_portion.cpp进行时间偏移估计
- 编译运行timestamp_correct.cpp实现时间戳校正
- 启动launch文件launch_cpp.launch实现融合消息的发送

---

# 目录结构 Project Structure

catkin_ws/
└── src/
    └── sensor_time_align/
        ├── CMakeLists.txt
        ├── include
        │   └── sensor_time_align
        ├── launch
        │   └── launch_cpp.launch
        ├── msg
        │   └── FusedState.msg
        ├── package.xml
        ├── scripts
        └── src
            ├── cam_w_norm_calculate.cpp
            ├── imu_w_norm_calculate.cpp
            ├── time_offset_calculate_portion.cpp
            ├── timestamp_correct.cpp
            ├── fusion_node.cpp
            └── publish.cpp


---

#  文件说明 File Description

##  顶层构建文件

### **CMakeLists.txt**
- **功能**：ROS1 catkin构建配置，声明依赖、编译消息、生成可执行文件。

### **package.xml**
- **功能**：ROS包的元信息，定义包名、版本、维护者和依赖。

---

## src

### **cam_w_norm_calculate.cpp**
- **功能**：估计相机角速度模长并保存。 
- **实现步骤**： 
  1. 遍历相邻的两帧图像数据记为若干组前后帧匹配对； 
  2. ORB算法提取相邻两帧图像的特征点和特征描述符； 
  3. 用BFMatcher进行特征匹配； 
  4. 提取匹配点的像素坐标；
  5. 估计本质矩阵；
  6. 解算得到得到旋转矩阵；
  7. 将旋转矩阵转换为旋转向量；
  8. 计算角速度并输出结果。

### **imu_w_norm_calculate.cpp**
- **功能**：计算角速度模长并保存。
- **实现步骤**： 
  1. 由imu三轴角速度原始数据直接计算得到模长。

### **time_offset_calculate_portion.cpp**
- **功能**：对比相机和IMU角速度模长序列，计算互相关从而估计时间偏移量。
- **实现步骤**： 
  1. 线性插值视觉数据到IMU序列。
  2. 设置时间偏移的搜索范围和搜索步长，逐个取值分别对各轴分量进行归一化互相关的计算。
  3. 求得各轴归一化互相关最大时的时间偏移估计值，将相关性最强的一轴数据的时间偏移估计值记为时间偏移量。
  
### **timestamp_correct.cpp**
- **功能**：修正相机时间戳，输出粗对齐后的时间戳。
- **实现步骤**： 
  1. 由估算出的偏移量对相机时间戳进行修正从而实现粗对齐。

### **publish.cpp**
- **功能**：ROS发布节点，读取数据发布话题 /cam_image 与 /imu_data。
- **实现步骤**： 
  1. 读取相机和imu的数据；
  2. 等待相机和IMU都有订阅者后开始发布话题； 
  3. 顺序发布时间在前的数据，时间相同的话先发布相机的再发布imu的；
  4. 最后等待2秒保证数据全部发布。 

### **fusion_node.cpp**
- **功能**：ROS融合节点，接收相机帧与区间内IMU数据，生成并发布融合数据。
- **实现步骤**： 
  1. 订阅相机和imu话题；
  2. 每一帧相机图像到来时发布一次融合消息，将当前到达的图像的时间戳作为融合消息的时间戳，按照FusedState.msg格式发布消息，图像是当前到达的图像，imu数据是[上一帧图像时间，本帧图像时间）内缓存的imu。

---

## msg

### **FusedState.msg**
- **功能**：自定义融合消息，包括：时间戳、图像、IMU角速度、加速度、IMU时间戳。

---

## launch

### **launch_cpp.launch**
- **功能**：一键启动 fusion_node 和 publish_node，自动完成发布与融合。

---

# 消息定义 Message
## FusedState.msg:
time stamp
string image_filename                           #相机图片文件名
sensor_msgs/Image image                         #图像数据
geometry_msgs/Vector3[] angular_velocity        #存储多个IMU的角速度数据
geometry_msgs/Vector3[] linear_acceleration     #存储多个IMU的加速度数据
float64[] imu_timestamp                         #存储多个IMU数据的时间戳

---

# 示例日志 Example
## 融合消息发布成功
[INFO] [1759214737.277746161]: Published fused data: 1403636763.613555.png with 10 IMU samples
[INFO] [1759214737.291120011]: Published fused data: 1403636763.663555.png with 10 IMU samples
[INFO] [1759214737.305273432]: Published fused data: 1403636763.713556.png with 10 IMU samples
[INFO] [1759214737.319367720]: Published fused data: 1403636763.763556.png with 10 IMU samples
[INFO] [1759214737.332512779]: Published fused data: 1403636763.813555.png with 10 IMU samples

## 发布节点发布结束
[INFO] [1759214737.340724093]: Data publish finished. Total cam: 3682, imu: 36820

## 融合节点发布结束
[INFO] [1759214749.469983085]: Fusion node finished. Total camera frames: 3682, total IMU samples fused: 36810

---

sensor_time_align
├── CMakeLists.txt
├── include
│   └── sensor_time_align
├── launch
│   ├── launch_cpp.launch
│   ├── node_output_topic.launch
│   ├── node_test_output_topic_data.launch
│   └── online_sensor_time_align.launch
├── msg
│   └── FusedState.msg
├── package.xml
├── scripts
└── src
    ├── cam_w_norm_calculate.cpp
    ├── fusion_node.cpp
    ├── image_raw_node.cpp
    ├── image_raw_node_test.cpp
    ├── imu_raw_node.cpp
    ├── imu_raw_node_test.cpp
    ├── imu_w_norm_calculate.cpp
    ├── publish.cpp
    ├── split.cpp
    ├── time_offset_calculate_portion.cpp
    ├── timestamp_correct.cpp
    └── timestamp_correct_node.cpp



详细实施步骤：
step1：创建工作区
在ROS中创建名为github_ws的工作区：
  创建工作区目录结构：
mkdir -p ~/github_ws/src
  初始化工作区：
cd ~/github_ws/src
catkin_init_workspace
  编译工作区：
cd ~/github_ws
catkin_make
  添加环境变量：
source devel/setup.bash
  
step2:创建功能包 sensor_time_align
catkin_create_pkg sensor_time_align roscpp rospy sensor_msgs geometry_msgs cv_bridge image_transport sensor_time_align std_msgs 

step3:离线粗对齐系统 offline_sensor_time_align 
  3.1至3.5推荐在vscode里面运行，也可以在ros包里面运行,下面内容写了在vscode里面运行需要作的调整，在ros包里运行已经配置好相关文件因此在工作区里rosrun可执行文件即可。
  3.1 对euroc数据集的MH_01_easy.bag进行分解得到数据集
  在vscode里面运行split.py实施分解，之后使用此步骤得到的数据集进行测试。
  如果是split.cpp的话操作步骤如下：
  #编译源码生成可执行文件：g++ split.cpp -o split
  #执行可执行文件：sudo ./split
  3.2 进行视觉角速度的计算
  如果你在vscode里面第一次调试运行使用opencv的库的cpp文件的话vscode找不到opencv会报错，此时需要修改.vscode里的tasks.json与c_cpp_properties.json让opencv可以被找到。
  头文件路径寻找命令：
  pkg-config --cflags opencv4
  库文件路径寻找命令：
  pkg-config --libs opencv4
  本项目opencv4.2.0的opencv头文件路径为/usr/include/opencv4库文件路径为/usr/lib/x86_64-linux-gnu因此修改本项目tasks.json为如下：
{
    "tasks": [
        {
            "type": "cppbuild",
            "label": "C/C++: g++ 生成带 OpenCV 的可执行文件",
            "command": "/usr/bin/g++",
            "args": [
                "-fdiagnostics-color=always",
                "-g",
                "-I/usr/include/opencv4",  //OpenCV头文件路径
                "${file}",  //待编译的源文件
                "-o",
                "${fileDirname}/${fileBasenameNoExtension}",  //输出可执行文件
                "-L/usr/lib/x86_64-linux-gnu",  //OpenCV库文件路径
                "-lopencv_core",
                "-lopencv_imgproc",
                "-lopencv_imgcodecs", 
                "-lopencv_highgui",
                "-lopencv_features2d", 
                "-lopencv_calib3d",     
                "-lopencv_video"       
            ],
            "options": {
                "cwd": "${fileDirname}"
            },
            "problemMatcher": [
                "$gcc"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "detail": "适配 OpenCV 的 C++ 编译任务，包含图像读写、特征提取、位姿估计等常用库"
        }
    ],
    "version": "2.0.0"
}
  修改本项目c_cpp_properties.json为如下：
{
  "configurations": [
    {
      "name": "linux-gcc-x64",
      "includePath": [
        "${workspaceFolder}/**",  //保留工作区自身的头文件路径
        "/usr/include/opencv4"    //OpenCV头文件路径
      ],
      "compilerPath": "/usr/bin/g++",
      "cStandard": "c11",
      "cppStandard": "c++11", 
      "intelliSenseMode": "linux-gcc-x64",
      "compilerArgs": [] 
    }
  ],
  "version": 4
}
  之后可以在vscode里面编译运行cam_w_norm_calculate.cpp即可完成视觉角速度的计算。原理是这样的：1.遍历相邻的两帧图像数据记为若干组前后帧匹配对； 2.ORB算法提取相邻两帧图像的特征点和特征描述符； 3.用BFMatcher进行特征匹配； 4.提取匹配点的像素坐标；5.估计本质矩阵，用RANSAC算法剔除外点；6.解算得到旋转矩阵；7.Rodrigues变换将旋转矩阵转为旋转向量；8.计算三轴视觉角速度以及角速度模长并输出结果。
  3.3 进行imu角速度模值的计算
  在vscode里面编译运行imu_w_norm_calculate.cpp即可完成imu角速度模值的计算。原理是这样的：1.由imu三轴角速度原始数据计算得到imu角速度模长并输出结果然后保存下来。
  3.4 进行时间偏移量b的计算
  在vscode里面编译运行time_offset_calculate_portion.cpp即可完成时间偏移量b的计算。时间偏移量b的计算有模值和三维角速度分量两种方法。此处选择依据三维角速度分量进行计算的方法，原理是这样的： 1.线性插值视觉数据到IMU数据序列；2. 设置时间偏移的搜索范围和搜索步长，对时间偏移逐个取值，分别对各轴分量进行归一化互相关的计算；3.求得各轴归一化互相关最大时的时间偏移估计值，将相关性最强的一轴数据的时间偏移估计值记为时间偏移量并输出。
  3.5 进行时间戳更正
  在vscode里面编译运行timestamp_correct.cpp即可完成时间戳更正。原理是这样的： 1.由估算出的偏移量对相机时间戳进行修正，实现粗对齐。
  
  用launch_cpp.launch可以对3.6和3.7环节统一启动，增加了发布节点延迟启动的环节，和数据发布环节中等待连接的步骤一起保障数据应发尽发不遗漏。
  3.6 数据的发布
  publish.cpp文件里面的内容实现数据发布的功能。文件规定了一个ROS发布节点pub_node，读取数据发布话题 /cam_image 与 /imu_data。原理是这样的：1.读取相机和imu信息，加入内容清洗环节，避免路径拼接错乱；2.等待相机和IMU都有订阅者连接后开始发布话题；3.按照时间顺序发布数据，先发布时间戳在前的数据，时间相同的话先发布相机的再发布imu的，同时在发布过程中检测数据是不是全部发送出去了，只要还有任意一种数据没发完，就继续循环发布；4.发布结束后等待2秒，确保数据全部发布不遗漏。 
  3.7 数据的融合
  fusion_node.cpp文件里的内容实现数据的融合。文件规定了一个ROS融合节点fusion_node，订阅相机帧与区间内IMU数据，生成并发布融合数据话题 /fused_topic。原理是这样的：1.订阅节点pub_node所发布的相机和imu话题；2.发布融合后的信息。每一帧相机信息到来时按照FusedState.msg格式发布一次融合消息，将当前到达的图像的时间戳作为融合消息的时间戳，融合消息中的图像是当前到达的图像，imu数据是[上一帧图像时间，本帧图像时间）内缓存的各个imu的三轴加速度、三轴角速度、原始时间戳。第一帧相机数据特殊处理不包含imu信息。
  之后可以对融合后发出的话题里面的数据进行检查验证。可以使用命令：
  rostopic echo /fused_topic
  可以看到消息里面的全部数据，但是通常由于图像数据太多了，在终端中往往只能观察到图像数据和区间内imu的角速度、加速度以及对应的原始时间戳，时间戳和图像名可以用以下命令来检测：
  rostopic echo /fused_topic | grep -E "timestamp|secs|nsecs|image_filename" 
  
step4:在线粗对齐系统 online_sensor_time_align 
  本项目所使用的传感器为一个单目相机和一个型号为MPU6050的传感器。
  4.1 相机的驱动
  本项目所使用的是一个单目相机，使用ros里面带有的usb_cam去进行驱动。
  对于ROS 1 Noetic版本安装驱动：
  sudo apt install ros-noetic-usb-cam
  source /opt/ros/noetic/setup.bash     # 加载ROS环境
  roslaunch usb_cam usb_cam-test.launch     # 启动相机
  之后每次重新打开设备的话，roscore之后进行下列操作即可：
  source /opt/ros/noetic/setup.bash
  roslaunch usb_cam usb_cam-test.launch
  4.2 相机数据的标准化
  4.3 IMU的驱动
  本项目使用的是型号为lubancat 4鲁班猫板卡和一个型号为MPU6050的IMU，使用I2C总线连接进行通信，本项目采用的是I2C6总线，IMU的VCC、GND、SCL、SDA引脚分别连接板卡40PIN接口的17、39、28、27号引脚。之后使用imu_raw_node.cpp文件或imu_raw_node_test.cpp文件对IMU进行驱动，imu_raw_node.cpp的功能是对imu实时采集到的数据进行话题发布，imu_raw_node_test.cpp的功能是对imu实时采集到的数据进行话题发布以及标准化格式保存。经过此步骤可以稳定得到imu数据的话题imu_raw用于之后的处理。
  4.4 在线时间戳粗对齐
  
  
  
  
  
  
  


  

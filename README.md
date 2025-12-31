# sensor_time_align

## 简介
一个多传感器时空联合标定系统，由时间对齐部分和空间标定部分组成,此外还包括一些硬件驱动节点和数据集文件分割节点，能够实现雷达、IMU、相机的时间对齐和空间标定，为多传感器数据融合后具体场景的应用算法提供更加精确的数据。  
时间对齐部分包括视觉惯性离线时间戳对齐、视觉惯性在线时间戳对齐、雷达惯性在线时间戳对齐、雷达视觉惯性在线时间戳对齐四个模块，每个模块使用不止一种算法去实现功能，从而可以根据硬件选型和应用场景的不同，因地制宜选择不同算法去实现时间对齐。此处离线和在线的区别在于离线时间戳对齐的数据来源是已经录制好的离线数据集文件，在线时间戳对齐的数据来源是传感器实时输出的信息。  
空间标定部分包括kalibr标定和ros张正友相机内参标定两部分。ros张正友相机内参标定部分可以迅速对单目相机进行标定，从而得到其内参；kalibr标定是高精度开源离线视觉惯性时空联合标定工具，可以对视觉传感器和IMU的内外参、时间偏移量进行精确的离线标定。  
将时间对齐部分和空间标定部分进行有序结合，能够高效、高精度实现多传感器时空联合标定。  

## 环境
- Ubuntu 20.04 
- ROS1 Noetic 
- OpenCV 4.2.0
- Minimum C++11

## 目录结构
```
sensor_time_align
    ├── CMakeLists.txt
    ├── include
    ├── launch
    │   ├── camera_imu_data.launch
    │   ├── camera_imu_data_save.launch
    │   ├── lidar_camera_imu_portion_omega.launch
    │   ├── offline.launch
    │   ├── online_camera_imu_align_norm.launch
    │   └── online_camera_imu_align_portion.launch
    ├── msg
    │   └── FusedState.msg
    ├── package.xml
    ├── README.md
    ├── scripts
    └── src
        ├── camera_imu_norm.cpp
        ├── camera_imu_portion.cpp
        ├── cam_w_norm_calculate.cpp
        ├── fusion_node.cpp
        ├── image_raw_node.cpp
        ├── image_raw_node_save.cpp
        ├── imu_raw_node.cpp
        ├── imu_raw_node_save.cpp
        ├── imu_w_norm_calculate.cpp
        ├── lidar_camera_imu.cpp
        ├── lidar_imu_distance.cpp
        ├── lidar_imu_omega.cpp
        ├── publish_node.cpp
        ├── split.cpp
        ├── time_offset_calculate_norm.cpp
        ├── time_offset_calculate_portion.cpp
        └── timestamp_correct.cpp
```

此目录结构是通过tree命令去生成的，指令如下：
```bash
tree -L 2
```
该指令可以查看2级目录结构，能看到包内的launch、src等文件夹以及其中的代码文件。  
tree命令的安装指令为：
```bash
sudo apt update && sudo apt install -y tree
```

## src文件原理解释及本项目详细实施步骤

### step1:创建工作区
在ROS中创建名为github_ws的工作区：  
创建工作区目录结构：
```bash
mkdir -p ~/github_ws/src
```
初始化工作区：
```bash
cd ~/github_ws/src
catkin_init_workspace
```
编译工作区：
```bash
cd ~/github_ws
catkin_make
```
添加环境变量：
```bash
source devel/setup.bash
```
  
### step2:创建功能包
```bash
catkin_create_pkg sensor_time_align roscpp rospy sensor_msgs geometry_msgs cv_bridge image_transport std_msgs
```

### step3:视觉惯性离线时间戳对齐
3.1至3.5推荐在vscode里面运行，也可以在ros包里面运行,下面内容写了在vscode里面运行需要作的调整，在ros包里运行时因为已经配置好了相关文件，所以在工作区里rosrun可执行文件即可。

#### 3.1 对euroc数据集的MH_01_easy.bag进行分解
使用split.cpp进行分割，操作步骤如下：  
编译源码生成可执行文件：
```bash
g++ split.cpp -o split
```
执行可执行文件：
```bash
sudo ./split
```

#### 3.2 进行视觉角速度的计算
如果在vscode里面第一次调试运行使用opencv的库的cpp文件的话，vscode找不到opencv会报错，此时需要修改.vscode里的tasks.json与c_cpp_properties.json使得opencv可以被找到。  
头文件路径寻找命令：
```bash
pkg-config --cflags opencv4
```
库文件路径寻找命令：
```bash
pkg-config --libs opencv4
```
本项目opencv4.2.0的opencv头文件路径为/usr/include/opencv4库文件路径为/usr/lib/x86_64-linux-gnu因此修改本项目tasks.json为如下内容：
```json
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
```
修改本项目c_cpp_properties.json为如下内容：
```json
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
```
之后可以在vscode里面编译运行cam_w_norm_calculate.cpp文件，即可完成视觉角速度的计算。原理是这样的：1.遍历相邻的两帧图像数据记为若干组前后帧匹配对； 2.ORB算法提取相邻两帧图像的特征点和特征描述符； 3.用BFMatcher进行特征匹配； 4.提取匹配点的像素坐标；5.估计本质矩阵，用RANSAC算法剔除外点；6.解算得到旋转矩阵；7.Rodrigues变换将旋转矩阵转为旋转向量；8.计算三轴视觉角速度以及角速度模长并输出结果。

#### 3.3 进行imu角速度模值的计算
在vscode里面编译运行imu_w_norm_calculate.cpp文件，即可完成imu角速度模值的计算。原理是这样的：1.由imu三轴角速度原始数据计算得到imu角速度模长并输出结果然后保存下来。

#### 3.4 进行时间偏移量b的计算
时间偏移量b的计算有模值和三维角速度分量两种方法。此处选择依据三维角速度分量进行计算的方法，在vscode里面编译运行time_offset_calculate_portion.cpp文件，即可完成时间偏移量b的计算。原理是这样的： 1.线性插值视觉数据到IMU数据序列；2. 设置时间偏移的搜索范围和搜索步长，对时间偏移逐个取值，分别对各轴分量进行归一化互相关的计算；3.求得各轴归一化互相关最大时的时间偏移估计值，将相关性最强的一轴数据的时间偏移估计值记为时间偏移量并输出。  
如果选择依据角速度模值进行计算的方法，需要在vscode里面编译运行time_offset_calculate_norm.cpp文件，即可完成时间偏移量b的计算。原理是这样的： 1.线性插值视觉数据到IMU数据序列；2. 设置时间偏移的搜索范围和搜索步长，对时间偏移逐个取值，对角速度模值进行归一化互相关的计算；3.求得归一化互相关最大时的时间偏移估计值，记为时间偏移量并输出。

#### 3.5 进行时间戳更正
在vscode里面编译运行timestamp_correct.cpp文件，即可完成时间戳更正。原理是这样的： 1.由估算出的偏移量对相机时间戳进行修正，实现粗对齐。  
  
#### 3.6 数据的发布
publish_node.cpp文件里面的内容可以实现数据发布的功能。该文件规定了一个ROS发布节点publish_node，此节点可以读取数据并发布话题 /cam_image 与 /imu_data。原理是这样的：1.读取相机和imu信息，加入内容清洗环节，避免路径拼接错乱；2.等待相机和IMU都有订阅者连接后开始发布话题；3.按照时间顺序发布数据，先发布时间戳在前的数据，时间相同的话先发布相机的再发布imu的，同时在发布过程中检测数据是不是全部发送出去了，只要还有任意一种数据没发完，就继续循环发布；4.发布结束后等待2秒，确保数据全部发布不遗漏。 

#### 3.7 数据的融合
fusion_node.cpp文件里的内容可以实现数据融合的功能。文件规定了一个ROS融合节点fusion_node，此节点可以订阅相机帧与区间内IMU数据，生成并发布融合数据话题 /fused_topic。原理是这样的：1.订阅节点publish_node所发布的相机和imu话题；2.发布融合后的信息。每一帧相机信息到来时按照FusedState.msg文件所定义的格式发布一次融合消息，将当前到达的图像的时间戳作为融合消息的时间戳，融合消息中的图像是当前到达的图像，imu数据是[上一帧图像时间，本帧图像时间）内缓存的各个imu的三轴加速度、三轴角速度、原始时间戳。对第一帧相机数据特殊处理，使其不包含imu信息。  
之后可以对融合后发出的话题里面的数据进行检查验证。可以使用命令：
```bash
rostopic echo /fused_topic
```
可以看到消息里面的全部数据，但是通常由于图像数据太多了，在终端中往往只能观察到图像数据和区间内imu的角速度、加速度以及对应的原始时间戳，时间戳和图像名可以用以下命令来检测：
```bash
rostopic echo /fused_topic | grep -E "timestamp|secs|nsecs|image_filename" 
```
也可以使用以下命令不去显示具体的图像像素数据而是简略显示：
```bash
rostopic echo --noarr /fused_topic 
```
  
### step4:视觉惯性在线时间戳对齐 
本项目所使用的传感器为一个单目相机和一个型号为MPU6050的IMU。

#### 4.1 相机的驱动
本项目所使用的是一个单目相机，使用ros里面带有的usb_cam去进行驱动。  
对于ROS 1 Noetic版本来说，需要先安装驱动：
```bash
sudo apt install ros-noetic-usb-cam
```
之后每次重新打开设备的话，加载noetic系统环境后，roscore之后进行下列操作即可：
```bash
roslaunch usb_cam usb_cam-test.launch
```

#### 4.2 相机数据的标准化
虽然上一步骤中相机所发布的话题也是带有时间戳的，但是本项目定义了话题名，且需要的是符合标准的相机数据，因此要进行相机数据的标准化，同时把相机输出的rgb8编码格式的图像转换为bgr8编码格式输出，便于opencv去处理。使用image_raw_node.cpp文件或image_raw_node_save.cpp文件对订阅到的相机话题进行相机数据的标准化，image_raw_node.cpp的功能是订阅相机原始图像话题，处理后重新发布带精确时间戳的bgr8编码格式的图像话题，image_raw_node_save.cpp的功能是订阅相机原始图像话题，处理后重新发布带精确时间戳的bgr8编码格式的图像话题并将图像帧及其名字、时间戳标准化保存下来形成数据集。

#### 4.3 IMU的驱动
本项目使用的是型号为lubancat 4鲁班猫板卡和一个型号为MPU6050的IMU，使用I2C总线连接进行通信，本项目采用的是I2C6总线，IMU的VCC、GND、SCL、SDA引脚分别连接板卡40PIN接口的17、39、28、27号引脚。之后使用imu_raw_node.cpp文件或imu_raw_node_save.cpp文件对IMU进行驱动，imu_raw_node.cpp的功能是对imu实时采集到的数据进行话题发布，imu_raw_node_save.cpp的功能是对imu实时采集到的数据进行话题发布以及标准化格式保存下来形成数据集。经过此步骤可以稳定得到imu数据的话题/imu_raw用于之后的处理。

#### 4.4 时间戳对齐
本项目的视觉惯性在线时间戳对齐环节可以使用camera_imu_portion.cpp文件或是camera_imu_norm.cpp文件，两个文件均可实现视觉惯性在线时间戳对齐的功能，区别在于计算时间偏移的时候camera_imu_portion.cpp文件使用三轴角速度而camera_imu_norm.cpp文件使用角速度模值去参与计算。在camera_imu_portion_node或camera_imu_norm_node节点中对相机数据话题和IMU数据话题进行订阅、处理，实现视觉惯性在线时间戳粗对齐的功能。原理大概是这样的：视觉惯性在线时间戳对齐分为校准阶段和修正阶段。校准阶段节点启动后，收集前3000帧图像数据和对应时间段的IMU数据，通过相邻图像帧的特征匹配与运动估计计算视觉角速度，之后使用三轴角速度或是角速度模值去与对应时间段缓存下来的IMU角速度进行运算，搜索得到最优时间偏移量bias；修正阶段以校准阶段得出的bias作为基准对在线收到的图像数据进行时间戳修正处理，并以一个新的话题进行发布，可用于kalibr标定以及后续的其它处理环节。

### step5:雷达惯性在线时间戳对齐

#### 5.1 雷达的驱动
下载雷达厂商提供的该型号雷达对应的驱动代码，在工作区进行编译，编译成与本系统平行的另一个功能包，之后对于本项目所采用的雷达来说需要使用以下操作驱动雷达：
```bash
roslaunch rplidar_ros rplidar_c1.launch
```

#### 5.2 IMU的驱动
此步骤与4.3节完全一致，可参考4.3完成IMU的驱动。

#### 5.3 时间戳对齐
本项目的雷达惯性在线时间戳对齐环节可以使用lidar_imu_omega.cpp文件或是lidar_imu_distance.cpp文件，两个文件均可实现激光雷达与IMU在线时间戳对齐的功能，区别在于计算雷达运动特征时 lidar_imu_omega.cpp文件通过帧间重心变化推导角速度，而lidar_imu_distance.cpp文件计算帧间重心偏移量。在lidar_imu_omega_node或lidar_imu_distance_node节点中对激光雷达数据话题以及IMU数据话题进行订阅、处理，实现雷达惯性在线时间戳对齐。原理大致如下：雷达惯性在线时间戳对齐分为校准阶段和修正阶段。校准阶段节点启动后，收集目标数量的雷达帧数据和对应时间段的IMU数据，通过相邻雷达帧的重心计算雷达运动特征，此时lidar_imu_omega.cpp计算角速度，lidar_imu_distance.cpp计算重心偏移量，这两种方法都可以得到雷达的运动特征，之后将雷达运动特征与对应时间段缓存的IMU角速度按轴提取进行运算，搜索得到最优时间偏移量bias；修正阶段以校准阶段得出的bias作为基准对在线收到的雷达数据进行时间戳修正处理，并以一个新的话题进行发布，可用于传感器空间标定以及后续的其它处理环节。

### step6:雷达视觉惯性在线时间戳对齐

#### 6.1 雷达的驱动
此步骤与5.1节完全一致，可参考5.1完成雷达的驱动。

#### 6.2 IMU的驱动
此步骤与4.3节完全一致，可参考4.3完成IMU的驱动。

#### 6.3 相机的驱动
此步骤与4.1节完全一致，可参考4.1完成相机的驱动。

#### 6.4 相机数据的标准化
此步骤与4.2节完全一致，可参考4.2完成相机数据的标准化。

#### 6.5 时间戳对齐
本项目的雷达视觉惯性在线时间戳对齐环节使用lidar_camera_imu.cpp文件，以IMU信息为基准，实现了雷达、相机与IMU的在线时间戳对齐功能。在lidar_camera_imu_node节点中通过订阅雷达、相机和IMU数据话题，分别实现雷达-IMU和相机-IMU的时间偏移校准，并发布修正后的雷达和相机数据。其原理分为校准阶段和修正阶段，且两组校准过程独立进行：相机-IMU校准阶段中，节点启动后收集前3000帧图像数据和对应时间段的IMU数据，通过相邻图像帧的ORB特征匹配，估计本质矩阵并分解得到旋转矩阵，计算视觉角速度，再将其与对应IMU角速度进行互相关分析，遍历搜索视觉惯性最优时间偏移量bias1；修正阶段则以bias1为基准，对后续图像数据进行时间戳修正并发布。雷达-IMU校准阶段收集前600帧雷达数据和对应时间段的IMU数据，通过相邻雷达帧重心偏移推导雷达角速度，与IMU角速度进行互相关分析，在设置的时间范围内设定固定步长搜索雷达惯性最优时间偏移量bias2；修正阶段以bias2为基准修正雷达时间戳后发布。修正后的数据可用于多传感器空间标定及后续融合处理，解决时间同步误差问题。

### step7:ROS单目相机内参张正友标定
本步骤是所用相机内参未知，想要标定获得相机内参时的独立步骤。标定相机内参的方法有很多，例如kalibr也可以标定，但在ROS里面用张正友标定法去标定相机内参速度是很快的，很适合对未知内参的相机进行标定。

#### 7.1 驱动单目相机
终端1启动ROS：
```bash
roscore
```
终端2驱动相机：
```bash
roslaunch usb_cam usb_cam-test.launch
```

#### 7.2 输入实际参数开始标定
终端3开始标定：
```bash
source /opt/ros/noetic/setup.bash
cd github_ws
source devel/setup.bash
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.0208 image:=/usb_cam/image_raw
```
参数说明：  
--size 11x8     # 角点的数量，是棋盘格的数量减1  
--square 0.0208     # 每个格子的大小，单位是米  
image:=/usb_cam/image_raw      # 图像topic所在的位置  
  
### step8:kalibr标定
kalibr是被广泛使用的多传感器时空标定工具包，包含多种模式，可以进行多种参数的高精度视觉惯性标定。本项目使用了kalibr标定中的相机标定kalibr_calibrate_cameras模式和相机IMU联合标定kalibr_calibrate_imu_camera模式去进行相机和IMU的标定。

#### 8.1 kalibr的编译
kalibr工具包的代码被开源到以下网址：  
https://github.com/ethz-asl/kalibr  
从Code按钮下选择Download ZIP选项，下载kalibr-master.zip之后解压，将解压所得的文件夹命名为kalibr，之后新建kalibr_ws工作区：
```bash
mkdir kalibr_ws
```
将kalibr文件夹放到kalibr_ws工作区的src文件夹内，然后进行以下操作去进行编译：
```bash
cd kalibr_ws
catkin_make
source devel/setup.bash
```
此时完成了对kalibr的编译以及编译成功后环境变量的加载。  
在编译过程中往往会出现无法编译成功kalibr工具的问题，体现为无法识别编译好的文件，对这个原因进行了分析，发现原因在于环境变量按照常规的方法添加无法被识别，找到解决方案是在无法识别时执行以下命令即可：
```bash
export PATH=$PATH:~/kalibr_ws/devel/lib/kalibr
source devel/setup.bash
```

#### 8.2 kalibr相机标定
自己先去录制离线bag，之后执行以下命令进行kalibr相机标定，会得到相机内参、多相机之间外参等结果数据。
```bash
kalibr_calibrate_cameras  \
  --bag ~/data/bw_data_test_2025-10-26-22-17-21.bag \
  --topics /image_timestamp_raw \
  --models pinhole-radtan \
  --target ~/kalibr_ws/checkerboard_board.yaml 
```

#### 8.3 kalibr相机IMU联合标定
使用离线bag执行以下命令进行kalibr相机IMU联合标定，得到外参、时间偏移等结果数据。
```bash
kalibr_calibrate_imu_camera \
  --bag ~/data/bw_data_test_2025-10-26-22-17-21.bag \
  --cam ~/data/bw_data_test_2025-10-26-22-17-21-camchain.yaml \
  --imu ~/kalibr_ws/imu.yaml \
  --target ~/kalibr_ws/checkerboard_board.yaml
```

#### 8.4 多次迭代kalibr相机IMU联合标定
将前一次kalibr相机IMU联合标定的结果参数作为后一次的参数的初始值进行下一次kalibr相机IMU联合标定，直到结果收敛为止，可以提高标定结果的精度。如果两次结果几乎一致，例如误差指标差别<5%，即可说明已经收敛，无需再继续。注意要准备多套离线bag文件，每一次用不同的离线bag文件。  
通常判断是否收敛的指标有这些：  
重投影误差(Reprojection error)是否进一步下降；  
IMU残差(gyro/accel residual)是否趋于稳定；  
时延结果(time offset)是否稳定；  
外参矩阵是否收敛(平移变化<1cm，旋转变化<0.5°)。  
若以上指标均趋于稳定那可以认为已经收敛，若差异较大就可进行再迭代或重新录制bag。

#### 8.5 视觉惯性时空联合标定
可以将时间戳粗对齐与kalibr标定去进行结合，利用时间戳粗对齐能够减少搜索空间、确保Kalibr优化更快收敛的优势和多次迭代能够提高标定结果精度的优势去优化标定流程，实现合理、高效的视觉惯性融合时空联合标定。  
本项目所使用的一种可行的视觉惯性时空联合标定方法如下：  
准备多套bag文件，其中有一个bag文件做了一次粗对齐，如果需要进行kalibr相机标定获得相机内参就先进行一次kalibr相机标定去得到相机内参，相机内参已知的话可以将该环节省略，之后用这个做了一次粗对齐的bag文件进行第一次kalibr相机IMU联合标定，然后将标定的结果中得到的各项有关参数作为下一次kalibr相机IMU联合标定的初始参数使用别的bag文件进行标定，以此类推进行迭代，直到得到收敛的结果为止。
  
## launch文件解释

### 1. offline.launch
这个launch文件可以启动视觉惯性离线时间戳对齐的发布节点和融合节点，实现IMU原始数据和修正后相机数据的发布、消息的融合以及融合后新消息话题的发布，即实现视觉惯性离线时间戳对齐ROS部分的全部流程。

### 2. camera_imu_data.launch
这个launch文件可以启动相机格式标准化节点和IMU驱动节点，实现相机数据和IMU数据的标准化在线话题发布。

### 3. camera_imu_data_save.launch
这个launch文件可以启动相机格式标准化节点和IMU驱动节点，实现相机数据和IMU数据的标准化在线话题发布以及数据的保存。

### 4. online_camera_imu_align_portion.launch
这个launch文件可以启动相机格式标准化节点、IMU驱动节点和视觉惯性在线时间戳对齐节点，实现视觉惯性在线时间戳对齐的全部功能和流程，使用三轴角速度去计算时间偏移。

### 5. online_camera_imu_align_norm.launch
这个launch文件可以启动相机格式标准化节点、IMU驱动节点和视觉惯性在线时间戳对齐节点，实现视觉惯性在线时间戳对齐的全部功能和流程，使用角速度模值去计算时间偏移。

### 6. lidar_camera_imu_portion_omega.launch
这个launch文件可以启动雷达视觉惯性在线时间戳对齐节点，实现雷达视觉惯性在线时间戳对齐的全部功能和流程，使用角速度去计算时间偏移。

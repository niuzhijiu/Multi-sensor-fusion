cmake_minimum_required(VERSION 3.10)
project(mid_lidar_imu_calib)

## 编译配置
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

## 查找ROS核心依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver2
  eigen_conversions
  message_generation
)

## 查找第三方库依赖
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## 声明catkin包配置
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs livox_ros_driver2 eigen_conversions message_runtime
  DEPENDS EIGEN3 YAML_CPP
)

## 头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/livox_ros_driver2/include
)

## 可执行文件
add_executable(mid_lidar_imu_calib_node src/mid_lidar_imu_calib_node.cpp)

## 等待livox消息生成完成
add_dependencies(mid_lidar_imu_calib_node ${catkin_EXPORTED_TARGETS} livox_ros_driver2_generate_messages_cpp)

## 链接所有依赖库
target_link_libraries(mid_lidar_imu_calib_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## 安装可执行文件
install(TARGETS mid_lidar_imu_calib_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

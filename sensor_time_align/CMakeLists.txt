cmake_minimum_required(VERSION 3.0.2)
project(sensor_time_align)

## C++11 支持
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
  livox_ros_driver2
)

## 查找系统依赖
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  filesystem  
  system      
)

## 消息文件
add_message_files(
  FILES
  FusedState.msg
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

## catkin 配置
catkin_package(
  CATKIN_DEPENDS 
    cv_bridge 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
    image_transport
    livox_ros_driver2
  DEPENDS 
    OpenCV 
    Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## 可执行文件
add_executable(split src/split.cpp)
add_executable(cam_w_norm_calculate src/cam_w_norm_calculate.cpp)
add_executable(imu_w_norm_calculate src/imu_w_norm_calculate.cpp)
add_executable(time_offset_calculate_portion src/time_offset_calculate_portion.cpp)
add_executable(time_offset_calculate_norm src/time_offset_calculate_norm.cpp)
add_executable(timestamp_correct src/timestamp_correct.cpp)
add_executable(publish_node src/publish_node.cpp)
add_executable(fusion_node src/fusion_node.cpp)
add_executable(image_raw_node src/image_raw_node.cpp)
add_executable(imu_raw_node src/imu_raw_node.cpp)
add_executable(image_raw_node_save src/image_raw_node_save.cpp)
add_executable(imu_raw_node_save src/imu_raw_node_save.cpp)
add_executable(camera_imu_portion src/camera_imu_portion.cpp)
add_executable(camera_imu_norm src/camera_imu_norm.cpp)
add_executable(lidar_imu_distance src/lidar_imu_distance.cpp)
add_executable(lidar_imu_omega src/lidar_imu_omega.cpp) 
add_executable(lidar_camera_imu src/lidar_camera_imu.cpp)
add_executable(imu_topic_convert src/imu_topic_convert.cpp)
add_executable(image_topic_convert src/image_topic_convert.cpp)
add_executable(mid_lidar_imu_distance src/mid_lidar_imu_distance.cpp)
add_executable(mid_lidar_imu_omega src/mid_lidar_imu_omega.cpp)
add_executable(mid_lidar_camera_imu src/mid_lidar_camera_imu.cpp)

## 保证消息生成在编译前完成
add_dependencies(split ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cam_w_norm_calculate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_w_norm_calculate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(time_offset_calculate_portion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(time_offset_calculate_norm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(timestamp_correct ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publish_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_raw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_raw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_raw_node_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_raw_node_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_imu_portion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_imu_norm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_imu_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_imu_omega ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lidar_camera_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_topic_convert ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_topic_convert ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mid_lidar_imu_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mid_lidar_imu_omega ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mid_lidar_camera_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库
target_link_libraries(split
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(cam_w_norm_calculate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(imu_w_norm_calculate
  ${catkin_LIBRARIES}
)
target_link_libraries(time_offset_calculate_portion
  ${catkin_LIBRARIES}
)
target_link_libraries(time_offset_calculate_norm
  ${catkin_LIBRARIES}
)
target_link_libraries(timestamp_correct
  ${catkin_LIBRARIES}
)
target_link_libraries(publish_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(fusion_node
  ${catkin_LIBRARIES}
)
target_link_libraries(image_raw_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(imu_raw_node
  ${catkin_LIBRARIES}
)
target_link_libraries(image_raw_node_save
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(imu_raw_node_save
  ${catkin_LIBRARIES}
)
target_link_libraries(camera_imu_portion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(camera_imu_norm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(lidar_imu_distance
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(lidar_imu_omega
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(lidar_camera_imu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(imu_topic_convert
  ${catkin_LIBRARIES}
)
target_link_libraries(image_topic_convert
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(mid_lidar_imu_distance
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(mid_lidar_imu_omega
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(mid_lidar_camera_imu
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

## 安装可执行文件
install(TARGETS
  split
  cam_w_norm_calculate
  imu_w_norm_calculate
  time_offset_calculate_portion 
  time_offset_calculate_norm
  timestamp_correct 
  publish_node
  fusion_node
  image_raw_node
  imu_raw_node
  image_raw_node_save
  imu_raw_node_save
  camera_imu_portion
  camera_imu_norm
  lidar_imu_distance
  lidar_imu_omega
  lidar_camera_imu
  imu_topic_convert
  image_topic_convert
  mid_lidar_imu_distance
  mid_lidar_imu_omega
  mid_lidar_camera_imu
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

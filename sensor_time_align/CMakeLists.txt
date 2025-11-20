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
add_executable(pub_node src/publish.cpp)
add_executable(fusion_node src/fusion_node.cpp)
add_executable(cam_w_norm_calculate src/cam_w_norm_calculate.cpp)
add_executable(time_offset_calculate_portion src/time_offset_calculate_portion.cpp)
add_executable(time_offset_calculate_norm src/time_offset_calculate_norm.cpp)
add_executable(imu_w_norm_calculate src/imu_w_norm_calculate.cpp)
add_executable(timestamp_correct src/timestamp_correct.cpp)
add_executable(split src/split.cpp)
add_executable(image_raw_node_test src/image_raw_node_test.cpp)
add_executable(imu_raw_node_test src/imu_raw_node_test.cpp)
add_executable(image_raw_node src/image_raw_node_test.cpp)
add_executable(imu_raw_node src/imu_raw_node_test.cpp)
add_executable(timestamp_correct_node_portion src/timestamp_correct_node_portion.cpp)
add_executable(timestamp_correct_node_norm src/timestamp_correct_node_norm.cpp)

## 保证消息生成在编译前完成
add_dependencies(pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cam_w_norm_calculate ${catkin_EXPORTED_TARGETS})
add_dependencies(time_offset_calculate_portion ${catkin_EXPORTED_TARGETS})
add_dependencies(time_offset_calculate_norm ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_w_norm_calculate ${catkin_EXPORTED_TARGETS})
add_dependencies(timestamp_correct ${catkin_EXPORTED_TARGETS})
add_dependencies(split ${catkin_EXPORTED_TARGETS})
add_dependencies(image_raw_node_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_raw_node_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_raw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_raw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(timestamp_correct_node_portion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(timestamp_correct_node_norm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库
target_link_libraries(pub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(fusion_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cam_w_norm_calculate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(time_offset_calculate_portion
  ${catkin_LIBRARIES}
)
target_link_libraries(time_offset_calculate_norm
  ${catkin_LIBRARIES}
)
target_link_libraries(imu_w_norm_calculate
  ${catkin_LIBRARIES}
)
target_link_libraries(timestamp_correct
  ${catkin_LIBRARIES}
)
target_link_libraries(split
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(image_raw_node_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(imu_raw_node_test
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
target_link_libraries(timestamp_correct_node_portion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(timestamp_correct_node_norm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

## 安装可执行文件
install(TARGETS 
  pub_node
  fusion_node 
  cam_w_norm_calculate 
  time_offset_calculate_portion 
  time_offset_calculate_norm
  imu_w_norm_calculate 
  timestamp_correct 
  split
  image_raw_node_test
  imu_raw_node_test
  image_raw_node
  imu_raw_node
  timestamp_correct_node_portion
  timestamp_correct_node_norm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

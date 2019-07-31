cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES curb_detection
#  CATKIN_DEPENDS pcl_ros roscpp rospy
#  DEPENDS system_lib
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lidar_start_nodelet
  apps/lidar_start_nodelet.cpp
)

target_link_libraries(lidar_start_nodelet
  ${catkin_LIBRARIES}
  liblidar_driver.a
  pcl_io
  pcl_common
)

cmake_minimum_required(VERSION 2.8.3)
project(pointrecorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosbag
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs rosbag
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(pointrecorder_node src/pointrecorder.cpp)

target_link_libraries(pointrecorder_node ${catkin_LIBRARIES})

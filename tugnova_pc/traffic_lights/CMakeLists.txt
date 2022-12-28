cmake_minimum_required(VERSION 2.8.3)
project(traffic_lights)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traffic_lights
#  CATKIN_DEPENDS autoware_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(traffic_light_detector src/traffic_lights.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(traffic_light_detector
  ${catkin_LIBRARIES}
)


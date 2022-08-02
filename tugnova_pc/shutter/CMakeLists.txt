cmake_minimum_required(VERSION 3.0.2)
project(shutter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
  #message_generation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shutter
  CATKIN_DEPENDS autoware_msgs roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(shutter_controller src/shutter_controller.cpp)

target_link_libraries(shutter_controller
  ${catkin_LIBRARIES}
)


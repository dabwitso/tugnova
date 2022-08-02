cmake_minimum_required(VERSION 3.0.2)
project(drive_stop_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drive_stop_controller
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(drive_stop_controller src/drive_stop_controller.cpp)

target_link_libraries(drive_stop_controller ${catkin_LIBRARIES})


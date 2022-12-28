cmake_minimum_required(VERSION 3.0.2)
project(pcd2int)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd_to_simple_signal
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(pcd2int src/pcd2int.cpp)

target_link_libraries(pcd2int
  ${catkin_LIBRARIES}
)


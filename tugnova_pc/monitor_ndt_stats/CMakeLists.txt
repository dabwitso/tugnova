cmake_minimum_required(VERSION 2.8.3)
project(monitor_ndt_stats)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  carctl_msgs
  autoware_msgs
  roscpp
  std_msgs
  udp_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES monitor_ndt_stats
#  CATKIN_DEPENDS autoware_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(ndt_stat_health src/ndt_stats_health.cpp)

target_link_libraries(ndt_stat_health
  ${catkin_LIBRARIES}
)

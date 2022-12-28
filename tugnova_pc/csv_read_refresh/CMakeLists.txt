cmake_minimum_required(VERSION 3.0.2)
project(csv_read_refresh)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csv_read_refresh
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(csv_refresh_clock src/csv_refresh_clock.cpp)
target_link_libraries(csv_refresh_clock
  ${catkin_LIBRARIES}
)


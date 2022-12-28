cmake_minimum_required(VERSION 3.0.2)
project(autoware_status)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autoware_status
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(status_code src/status_code.cpp)

target_link_libraries(status_code
  ${catkin_LIBRARIES}
)


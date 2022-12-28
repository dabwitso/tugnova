cmake_minimum_required(VERSION 2.8.3)
project(lamp_checker)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  communication_msgs
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lamp_check
#  CATKIN_DEPENDS autoware_msgs communication_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(lamp_checker src/lamp_checker.cpp)

target_link_libraries(lamp_checker
  ${catkin_LIBRARIES}
)


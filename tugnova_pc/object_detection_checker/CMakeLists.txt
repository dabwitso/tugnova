cmake_minimum_required(VERSION 2.8.3)
project(object_detection_checker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection_checker
#  CATKIN_DEPENDS autoware_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(object_detection_checker src/object_detection_checker.cpp)

target_link_libraries(object_detection_checker
  ${catkin_LIBRARIES}
)

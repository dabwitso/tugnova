cmake_minimum_required(VERSION 3.0.2)
project(sound_box)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
  udp_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sound_box
#  CATKIN_DEPENDS autoware_msgs roscpp std_msgs udp_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(sound_box src/sound_box.cpp)

target_link_libraries(sound_box
  ${catkin_LIBRARIES}
)


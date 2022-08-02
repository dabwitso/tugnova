cmake_minimum_required(VERSION 2.8.3)
project(mabx_connector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  carctl_msgs
  udp_msgs
)

find_library(udp_connector REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  carctl_msgs
  udp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${carctl_msgs_INCLUDE_DIRS}
  ${udp_connector_SOURCE_DIR}/include
)

add_definitions(-std=c++11)

add_executable(mabx_sender src/mabx_sender.cpp src/mabx_sender_node.cpp)
target_link_libraries(mabx_sender udp_connector ${catkin_LIBRARIES})
add_dependencies(mabx_sender ${catkin_EXPORTED_TARGETS})

add_executable(mabx_receiver src/mabx_receiver.cpp src/mabx_receiver_node.cpp)
target_link_libraries(mabx_receiver udp_connector ${catkin_LIBRARIES})
add_dependencies(mabx_receiver ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
cmake_minimum_required(VERSION 2.8.3)
project(battery_ctlvehicle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  carctl_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  carctl_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${carctl_msgs_INCLUDE_DIRS}
  ${battery_checker_SOURCE_DIR}/include
)

add_definitions(-std=c++11)

add_executable(
  battery_ctlvehicle 
  src/battery_ctlvehicle.cpp 
)
target_link_libraries(battery_ctlvehicle ${catkin_LIBRARIES})
add_dependencies(battery_ctlvehicle ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
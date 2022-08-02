cmake_minimum_required(VERSION 2.8.3)
project(vehicle_waypoint_distance_checker)

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
)

add_definitions(-std=c++11)

add_executable(
  vehicle_waypoint_distance_checker 
  src/vehicle_waypoint_distance_checker.cpp 
)
target_link_libraries(vehicle_waypoint_distance_checker ${catkin_LIBRARIES})
add_dependencies(vehicle_waypoint_distance_checker ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
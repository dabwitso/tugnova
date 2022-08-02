cmake_minimum_required(VERSION 2.8.3)
project(waypoint_editor)

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
  waypoint_editor 
  src/waypoint_editor.cpp 
  src/waypoint_editor_node.cpp
  src/waypoint_editor_param.cpp 
  src/waypoint_editor_data.cpp
  src/waypoint_editor_curve_detection.cpp
  src/waypoint_editor_output_csv.cpp
  src/waypoint_editor_hotspot.cpp
)
target_link_libraries(waypoint_editor ${catkin_LIBRARIES})
add_dependencies(waypoint_editor ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
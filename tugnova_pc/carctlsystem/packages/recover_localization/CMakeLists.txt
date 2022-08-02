cmake_minimum_required(VERSION 2.8.3)
project(recover_localization)

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
  recover_localization 
  src/recover_localization.cpp 
  src/recover_localization_node.cpp 
  src/recover_localization_data.cpp 
  src/recover_localization_statistics.cpp
)
target_link_libraries(recover_localization ${catkin_LIBRARIES})
add_dependencies(recover_localization ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
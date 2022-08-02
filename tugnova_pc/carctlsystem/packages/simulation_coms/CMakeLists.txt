cmake_minimum_required(VERSION 2.8.3)
project(simulation_coms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  udp_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  udp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${carctl_msgs_INCLUDE_DIRS}
)

add_executable(simulation_coms src/simulation_coms.cpp)
target_link_libraries(simulation_coms ${catkin_LIBRARIES})
add_dependencies(simulation_coms ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
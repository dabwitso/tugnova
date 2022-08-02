cmake_minimum_required(VERSION 2.8.3)
project(communication)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-lpthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  communication_msgs
)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs communication_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
   src/server.py
   #src/dumnmy_tugnova.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



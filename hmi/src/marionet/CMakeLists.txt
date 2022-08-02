cmake_minimum_required(VERSION 3.0.2)
project(marionet)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation
)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MarionetteMessage.msg
#   Message2.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
 )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marionet
  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   src/publisher.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


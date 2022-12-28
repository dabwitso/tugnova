cmake_minimum_required(VERSION 2.8.3)
project(communication_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

 add_message_files(
   FILES
   RouteInfo.msg
   RouteResponse.msg
   Stats.msg
   MarionetteMessage.msg
   LampState.msg
   TugnovaGPS.msg
   PlcConveyor.msg
 )


 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)


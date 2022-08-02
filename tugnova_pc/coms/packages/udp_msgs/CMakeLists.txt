cmake_minimum_required(VERSION 2.8.3)
project(udp_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_message_files(
    FILES 
    UdpControlPacket.msg 
    UdpSensorPacket.msg
    UdpSensorPacket_GreenComs.msg 
    CarInfo.msg 
    ControlInfo.msg
    LocalizationInfo.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Declare a catkin package 
catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs
    geometry_msgs
    sensor_msgs
)


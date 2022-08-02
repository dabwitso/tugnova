cmake_minimum_required(VERSION 2.8.3)
project(carctl_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_message_files(
    FILES
    RouteCheck.msg
    monitor_status.msg
    phone_msg.msg
    waiting_msg.msg
    MabxSenderPacket.msg
    MabxReceiverPacket.msg
    wp_edit_begin_msg.msg
    wp_edit_finish_msg.msg
    wp_edit_stop_msg.msg
    wp_edit_split_msg.msg
    wp_edit_pause_msg.msg
    output_target.msg
    wp_edit_output_msg.msg
    wp_edit_2d_msg.msg
    wp_edit_hotspot_msg.msg
    emergency_status.msg
    battery_scale.msg
    battery_status.msg
    battery_checker_command.msg
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


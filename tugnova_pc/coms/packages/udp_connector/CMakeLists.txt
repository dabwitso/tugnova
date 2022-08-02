cmake_minimum_required(VERSION 2.8.3)
project(udp_connector)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf geometry_msgs autoware_msgs tablet_socket_msgs udp_msgs)

## Declare a catkin package 
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs geometry_msgs autoware_msgs tablet_socket_msgs udp_msgs visualization_msgs tf message_filters udp_msgs
)

## Build 
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(include)

add_executable(plc_sender nodes/plc/plc_sender.cpp nodes/plc/plc_sender_node.cpp)
target_link_libraries(plc_sender udp_connector ${catkin_LIBRARIES})
add_dependencies(plc_sender ${catkin_EXPORTED_TARGETS})

add_executable(plc_receiver nodes/plc/plc_receiver.cpp nodes/plc/plc_receiver_node.cpp)
target_link_libraries(plc_receiver udp_connector ${catkin_LIBRARIES})
add_dependencies(plc_receiver ${catkin_EXPORTED_TARGETS})

add_executable(comsctrl_sender nodes/comsctrl/comsctrl_sender.cpp nodes/comsctrl/comsctrl_sender_node.cpp)
target_link_libraries(comsctrl_sender udp_connector ${catkin_LIBRARIES})
add_dependencies(comsctrl_sender ${catkin_EXPORTED_TARGETS})

add_executable(comsctrl_receiver nodes/comsctrl/comsctrl_receiver.cpp nodes/comsctrl/comsctrl_receiver_node.cpp)
target_link_libraries(comsctrl_receiver udp_connector ${catkin_LIBRARIES})
add_dependencies(comsctrl_receiver ${catkin_EXPORTED_TARGETS})
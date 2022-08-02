cmake_minimum_required(VERSION 2.8.3)
project(udp_converter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf geometry_msgs autoware_msgs tablet_socket_msgs udp_msgs)

## Declare a catkin package 
catkin_package(
    DEPENDS system_lib 
    CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs geometry_msgs autoware_msgs tablet_socket_msgs udp_msgs visualization_msgs tf message_filters
)

## Build 
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(plc_converter src/plc_converter.cpp)
target_link_libraries(plc_converter ${catkin_LIBRARIES})
add_dependencies(plc_converter ${catkin_EXPORTED_TARGETS})



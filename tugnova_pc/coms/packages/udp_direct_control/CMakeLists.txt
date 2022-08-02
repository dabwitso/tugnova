cmake_minimum_required(VERSION 2.8.3)
project(udp_direct_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

## Declare a catkin package 
catkin_package(
    DEPENDS system_lib 
    CATKIN_DEPENDS roscpp)

## Build 
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(udp_direct_control src/udp_direct_control.cpp)
target_link_libraries(udp_direct_control ${catkin_LIBRARIES})
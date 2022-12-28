cmake_minimum_required(VERSION 3.0.2)
project(hotspot_trigger)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hotspot_trigger
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(hotspot_trigger src/hotspot_trigger.cpp)

target_link_libraries(hotspot_trigger
  ${catkin_LIBRARIES}
)


cmake_minimum_required(VERSION 2.8.3)
project(battery_meter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  udp_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES battery_meter
#  CATKIN_DEPENDS roscpp std_msgs udp_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#add_dependencies(${PROJECT_NAME} ${udp_msgs_EXPORTED_TARGETS})

add_executable(battery_meter src/battery_meter.cpp)

target_link_libraries(battery_meter ${catkin_LIBRARIES})


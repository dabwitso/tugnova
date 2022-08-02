cmake_minimum_required(VERSION 2.8.3)
project(drive_mode_controller)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-lpthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  geometry_msgs
  communication_msgs
  )



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs autoware_msgs geometry_msgs carctl_msgs communication_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(drive_controller src/drive_controller.cpp)

#add_dependencies(${PROJECT_NAME}
#  ${autoware_msgs_EXPORTED_TARGETS}
#  ${communication_msgs_EXPORTED_TARGETS}
#  )

target_link_libraries(drive_controller ${catkin_LIBRARIES})



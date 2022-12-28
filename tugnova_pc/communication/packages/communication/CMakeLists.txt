cmake_minimum_required(VERSION 2.8.3)
project(communication)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-lpthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  autoware_msgs
  carctl_msgs
  geometry_msgs
  communication_msgs
  udp_msgs
  )



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs autoware_msgs geometry_msgs carctl_msgs communication_msgs carctl_msgs udp_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(waypoint_selector src/waypoint_selector.cpp)
add_executable(waypoint_job_handler src/waypoint_job_handler.cpp)
add_executable(tagnova_hmi_transfer src/tagnova_hmi_transfer.cpp)

#add_dependencies(${PROJECT_NAME} ${communication_msgs_EXPORTED_TARGETS})

target_link_libraries(waypoint_selector ${catkin_LIBRARIES})
target_link_libraries(waypoint_job_handler ${catkin_LIBRARIES})
target_link_libraries(tagnova_hmi_transfer ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  #src/server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



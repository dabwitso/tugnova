cmake_minimum_required(VERSION 2.8.3)
project(monitoring_health)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  carctl_msgs
  udp_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  carctl_msgs
  udp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${carctl_msgs_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/ping_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/phone_notification.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(check_plc src/check_plc.cpp)
target_link_libraries(check_plc ${catkin_LIBRARIES})
add_dependencies(check_plc ${catkin_EXPORTED_TARGETS})

add_executable(monitoring_notification src/monitoring_notification.cpp)
target_link_libraries(monitoring_notification ${catkin_LIBRARIES})
add_dependencies(monitoring_notification ${catkin_EXPORTED_TARGETS})


#TODO INSTALL ALL THE WAVs, MODIFY SCRIPT
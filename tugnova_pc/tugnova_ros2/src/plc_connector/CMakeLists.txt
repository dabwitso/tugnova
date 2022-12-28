cmake_minimum_required(VERSION 3.8)
project(plc_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(udp_msgs REQUIRED)

add_executable(plc_sender src/plc_sender.cpp)
ament_target_dependencies(plc_sender rclcpp std_msgs udp_msgs)

add_executable(plc_receiver src/plc_receiver.cpp)
ament_target_dependencies(plc_receiver rclcpp std_msgs udp_msgs)

target_include_directories(plc_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(plc_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS plc_receiver plc_sender
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

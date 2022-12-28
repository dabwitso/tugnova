cmake_minimum_required(VERSION 3.8)
project(udp_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(udp_msgs REQUIRED)


#add_subdirectory(include/udp_connector)

## Build
include_directories( include )

add_executable(plc_sender src/plc_sender.cpp src/plc_sender_node.cpp src/udp_connector.cpp)
ament_target_dependencies(plc_sender rclcpp std_msgs udp_msgs)

add_executable(plc_receiver src/plc_receiver.cpp src/plc_receiver_node.cpp src/udp_connector.cpp)
ament_target_dependencies(plc_receiver rclcpp std_msgs udp_msgs)


target_include_directories(plc_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(plc_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS plc_sender plc_receiver
  DESTINATION lib/${PROJECT_NAME})


# add_library(plc_sender SHARED nodes/plc/plc_sender.cpp nodes/plc/plc_sender_node.cpp)
# add_library(listener_component SHARED src/listener_component.cpp)
# target_compile_definitions(talker_component PRIVATE "COMPOSITION_BUILDING_DLL")
# target_compile_definitions(listener_component PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)
# ament_target_dependencies(listener_component rclcpp rclcpp_components std_msgs)
# rclcpp_components_register_nodes(talker_component "cpp_pubsub::TalkerComponent")
# rclcpp_components_register_nodes(listener_component "cpp_pubsub::ListenerComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

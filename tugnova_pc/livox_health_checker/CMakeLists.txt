cmake_minimum_required(VERSION 2.8.3)
project(livox_health_checker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_health_checker
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_checker src/health_checker.cpp)

target_link_libraries(livox_checker
  ${catkin_LIBRARIES}
)


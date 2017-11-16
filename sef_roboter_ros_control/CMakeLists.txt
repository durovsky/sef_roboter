cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_ros_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sef_roboter_msgs
  ros_control_boilerplate)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sef_roboter_msgs
    ros_control_boilerplate
  INCLUDE_DIRS
    include   
)

include_directories(include ${CATKIN_INCLUDE_DIRS} ${ros_control_boilerplate_INCLUDE_DIRS})

# SefRoboter Hardware Interface
add_library(sef_roboter_hw_interface
  src/sef_roboter_hw_interface.cpp
)
target_link_libraries(sef_roboter_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(sef_roboter_hw_main src/sef_roboter_hw_main.cpp)
target_link_libraries(sef_roboter_hw_main
  sef_roboter_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)


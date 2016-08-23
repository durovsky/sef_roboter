cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_ros_control)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_control_boilerplate)

include_directories(${CATKIN_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)
include_directories(${ros_control_boilerplate_INCLUDE_DIRS})

catkin_package()

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


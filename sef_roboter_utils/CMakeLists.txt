cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    trajectory_msgs
    control_msgs
    std_msgs
    cmake_modules
    tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    trajectory_msgs
    control_msgs
    std_msgs
    cmake_modules
    tf
 )

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_executable(
  control_msg_parser
  src/control_msg_parser.cpp)

add_executable(
  moveit_interface
  src/moveit_interface.cpp)

add_executable(
  tango_tool0_aligner
  src/tango_tool0_aligner.cpp)

add_executable(
  button_plan_execute
  src/button_plan_execute.cpp)

add_executable(
  real_time_control
  src/real_time_control.cpp)

add_executable(
  accuracy_measurement
  src/accuracy_measurement.cpp)

add_dependencies(
  control_msg_parser
  ${catkin_EXPORTED_TARGETS})

add_dependencies(
  moveit_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(
  button_plan_execute
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(
  real_time_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  control_msg_parser
  ${catkin_LIBRARIES})

target_link_libraries(
  moveit_interface
  ${catkin_LIBRARIES})

target_link_libraries(
  tango_tool0_aligner
  ${catkin_LIBRARIES})

target_link_libraries(
  button_plan_execute
  ${catkin_LIBRARIES})

target_link_libraries(
  real_time_control
  ${catkin_LIBRARIES})

target_link_libraries(
  accuracy_measurement
  ${catkin_LIBRARIES})

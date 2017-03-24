cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_utils)

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

catkin_package()

find_package(Eigen REQUIRED)

include_directories(
  SYSTEM ${EIGEN_INCLUDE_DIRS}
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

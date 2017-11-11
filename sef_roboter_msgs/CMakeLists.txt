cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

add_service_files(
  FILES
  homing.srv
  reference_joint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
)

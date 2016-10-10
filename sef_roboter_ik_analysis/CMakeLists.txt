cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_ik_analysis)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  cmake_modules
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sef_roboter_ik_analysis src/sef_roboter_ik_analysis.cpp)
target_link_libraries(sef_roboter_ik_analysis ${catkin_LIBRARIES})

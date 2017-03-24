cmake_minimum_required(VERSION 2.8.3)
project(sef_roboter_joint_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui Qtsvg)

catkin_package()

include(${QT_USE_FILE})

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/Sources/)
include_directories(${PROJECT_SOURCE_DIR}/Headers/)
include_directories(${PROJECT_SOURCE_DIR}/Forms/)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

SET(SOURCES ${PROJECT_SOURCE_DIR}/Sources/main.cpp
	    ${PROJECT_SOURCE_DIR}/Sources/mainwindow.cpp)
SET(HEADERS ${PROJECT_SOURCE_DIR}/Headers/mainwindow.h)

SET(FORMS ${PROJECT_SOURCE_DIR}/Forms/mainwindow.ui)

QT4_WRAP_CPP(HEADERS_MOC ${HEADERS})
QT4_WRAP_UI(FORMS_HEADERS ${FORMS})



add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${FORMS} ${HEADERS_MOC} ${FORMS_HEADERS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES})

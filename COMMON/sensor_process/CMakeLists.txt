cmake_minimum_required(VERSION 2.8.3)
project(sensor_process)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sailboat_message
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_process
#  CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs
#  DEPENDS system_lib
)


include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS})

set( sensor_process_SRC
        lib/CSensorProcess.cpp)

add_library( ${PROJECT_NAME} ${sensor_process_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(Sensor_Process src/Sensor_Process.cpp)
target_link_libraries(Sensor_Process ${catkin_LIBRARIES} sensor_process)
add_dependencies(Sensor_Process sailboat_message_generate_message_cpp)

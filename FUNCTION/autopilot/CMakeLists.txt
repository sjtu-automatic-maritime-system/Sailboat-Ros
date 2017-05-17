cmake_minimum_required(VERSION 2.8.3)
project(autopilot)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  sailboat_message
  std_msgs
        autopilot_lib
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES autopilot
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs autopilot_lib
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})



add_executable(autopilot_ver1 src/AutoPilot.cpp)
target_link_libraries(autopilot_ver1 ${catkin_LIBRARIES})
add_dependencies(autopilot_ver1 autolipot_gencfg sailboat_message_generate_message_cpp mach_onboat_generate_message_cpp)


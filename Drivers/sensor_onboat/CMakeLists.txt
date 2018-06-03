cmake_minimum_required(VERSION 2.8.3)
project(sensor_onboat)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sailboat_message
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sensor_onboat
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)



include_directories(include ${catkin_INCLUDE_DIRS})



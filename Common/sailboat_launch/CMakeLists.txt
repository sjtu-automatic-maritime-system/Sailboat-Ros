cmake_minimum_required(VERSION 2.8.3)
project(sailboat_launch)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sailboat_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)




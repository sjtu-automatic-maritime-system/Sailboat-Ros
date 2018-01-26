cmake_minimum_required(VERSION 2.8.3)
project(sailboat_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(catkin_package_DEPENDENCIES
    gazebo_plugins
    gazebo_ros
    roscpp
    sensor_msgs
    tf
    hector_gazebo_plugins
    )

find_package(catkin REQUIRED COMPONENTS
  ${catkin_package_DEPENDENCIES}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sailboat_gazebo
  CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


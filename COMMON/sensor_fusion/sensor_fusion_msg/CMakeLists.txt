cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_msg)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
        FILES
        GpsMeasurement.msg
        LaserMeasurement.msg
        RaderMeasurement.msg
        KFResult.msg
        GpsKF.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_fusion_msg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


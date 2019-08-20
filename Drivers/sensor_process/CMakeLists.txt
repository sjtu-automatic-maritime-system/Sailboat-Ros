cmake_minimum_required(VERSION 2.8.3)
project(sensor_process)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sailboat_message
        std_msgs
        message_filters
        )

catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES sensor_process
        CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs message_filters
        #DEPENDS system_lib
)



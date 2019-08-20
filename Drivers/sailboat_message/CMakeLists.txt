cmake_minimum_required(VERSION 2.8.3)
project(sailboat_message)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        FILES

        #driver
        Arduino_msg.msg
        WTST_Pro_msg.msg
        Mach_msg.msg
        Sensor_all_msg.msg
        Sensor_msg.msg
        WTST_msg.msg
        Ahrs_msg.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        # INCLUDE_DIRS include
        #LIBRARIES sailboat_message
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        #DEPENDS system_lib
)


cmake_minimum_required(VERSION 2.8.3)
project(mach_onboat)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sailboat_message
        std_msgs
        )


catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES mach_onboat
        CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs
        #DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

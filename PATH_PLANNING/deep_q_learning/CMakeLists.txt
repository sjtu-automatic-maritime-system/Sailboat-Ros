cmake_minimum_required(VERSION 2.8.3)
project(deep_q_learning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        dynamic_reconfigure
        roscpp
        rospy
        sailboat_message
        std_msgs
        nav_msgs)

catkin_package(
        #INCLUDE_DIRS include
        LIBRARIES deep_q_learning
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs
)

###########
## Build ##
###########

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})


add_executable(rviz_marker src/rviz_marker.cpp)
target_link_libraries(rviz_marker ${catkin_LIBRARIES})

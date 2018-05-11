cmake_minimum_required(VERSION 2.8.3)
project(sailboat_vpp)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sailboat_message
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sailboat_vpp
        CATKIN_DEPENDS roscpp rospy std_msgs sailboat_message
        #DEPENDS system_lib
)
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})

set(sailboat_vpp_SRC
        lib/CDynamicVPP.cpp
        )

add_library(${PROJECT_NAME} ${sailboat_vpp_SRC})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(vpp_test_demo demo/DynamicVppTestDemo.cpp)
target_link_libraries(vpp_test_demo ${catkin_LIBRARIES} sailboat_vpp)
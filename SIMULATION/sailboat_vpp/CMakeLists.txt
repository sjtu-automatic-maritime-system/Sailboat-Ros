cmake_minimum_required(VERSION 2.8.3)
project(sailboat_vpp)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sailboat_message
        )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sailboat_vpp
        CATKIN_DEPENDS roscpp rospy std_msgs sailboat_message
        DEPENDS system_lib
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
cmake_minimum_required(VERSION 2.8.3)
project(avoidance)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        roscpp
        rospy
        std_msgs
        sailboat_message
        tld_msgs
        )

generate_dynamic_reconfigure_options(
        cfg/avoidance.cfg
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES avoidance
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs sailboat_message tld_msgs
        #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})



set( avoidance_SRC
src/avoidance_main.cpp
src/collision_avoidance.cpp
src/rt_nonfinite.cpp
src/rtGetInf.cpp
src/rtGetNaN.cpp
)

add_executable( ${PROJECT_NAME} ${avoidance_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} avoidance_gencfg)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )


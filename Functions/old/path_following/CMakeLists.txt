cmake_minimum_required(VERSION 2.8.3)
project(path_following)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        roscpp
        rospy
        sailboat_message
        std_msgs
        )


generate_dynamic_reconfigure_options(
        cfg/path_following_point.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)

catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES path_following
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs
        #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})


set( path_following_SRC
        src/PathFollowing.cpp
        src/CPathFollowingVer1.cpp
        )

add_executable( ${PROJECT_NAME} ${path_following_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} path_following_gencfg)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )


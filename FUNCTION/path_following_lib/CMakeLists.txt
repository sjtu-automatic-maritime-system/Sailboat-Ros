cmake_minimum_required(VERSION 2.8.3)
project(path_following_lib)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  sailboat_message
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_following_lib
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})

set( path_following_lib_SRC
        src/CPathFollowingVer1.cpp
        )

add_library( ${PROJECT_NAME} ${path_following_lib_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

#add_executable(path_following_test_demo demo/pathFollowingTestDemo.cpp)
#target_link_libraries(path_following_test_demo ${catkin_LIBRARIES} path_following_lib)

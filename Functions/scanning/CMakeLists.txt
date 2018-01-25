cmake_minimum_required(VERSION 2.8.3)
project(scanning)

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


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/scanning.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scanning
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs
  #DEPENDS system_lib
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

set( scanning_SRC
        src/scanning_main.cpp
        src/rt_nonfinite.cpp
        src/rtGetInf.cpp
        src/rtGetNaN.cpp
        src/scanning.cpp
        )

add_executable( ${PROJECT_NAME} ${scanning_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} scanning_gencfg)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

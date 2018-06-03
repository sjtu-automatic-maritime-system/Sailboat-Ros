cmake_minimum_required(VERSION 2.8.3)
project(commander)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  sailboat_message
        command_msgs
)

generate_dynamic_reconfigure_options(
        cfg/commander.cfg
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES avoidance
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message command_msgs
        #DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})

set( commander_SRC
        #src/commander.cpp
        #src/commander_main.cpp
        )

#add_executable( ${PROJECT_NAME} ${commander_SRC} )
#add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} commander_gencfg)
#target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

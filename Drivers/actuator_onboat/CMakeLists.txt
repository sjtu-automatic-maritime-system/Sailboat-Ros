cmake_minimum_required(VERSION 2.8.3)
project(actuator_onboat)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sailboat_message
        std_msgs
        )


catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES actuator_onboat
        CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs
        #DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

set( actuator_onboat_SRC
        src/base_control.cpp
        )

add_executable(base_control ${actuator_onboat_SRC})
add_dependencies(base_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_control ${catkin_LIBRARIES})


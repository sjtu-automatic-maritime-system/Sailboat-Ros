cmake_minimum_required(VERSION 2.8.3)
project(sailboat_simulation)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sailboat_message
        math_tool_lib
        sailboat_simulation_lib
        )


catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES sailboat_simulation
        CATKIN_DEPENDS roscpp rospy std_msgs sailboat_simulation_lib sailboat_message math_tool_lib
        #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})


add_executable(sailboat_simulation_ver1 src/Sailboat_Simulation.cpp)
target_link_libraries(sailboat_simulation_ver1 ${catkin_LIBRARIES})
#math_tool_lib sailboat_simulation_lib
add_dependencies(sailboat_simulation_ver1 ${catkin_EXPORTED_TARGETS})


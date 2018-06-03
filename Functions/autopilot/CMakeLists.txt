cmake_minimum_required(VERSION 2.8.3)
project(autopilot)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        roscpp
        rospy
        sailboat_message
        std_msgs
        math_tool_lib
        )
        
generate_dynamic_reconfigure_options(
        cfg/PID_Adjustment.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES autopilot
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs math_tool_lib
        #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})

set(autopilot_SRC
        src/AutoPilot.cpp
        src/CAutopilotVer1.cpp
        src/CSailCtrl.cpp
        )


add_executable(${PROJECT_NAME} ${autopilot_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} autopilot_gencfg)
#autolipot_gencfg

cmake_minimum_required(VERSION 2.8.3)
project(command_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
    FILES
    #cmd
    cmd_battery_status.msg
    cmd_camera_status.msg
    cmd_commander_cfg_status.msg
    cmd_commander_status.msg
    cmd_communication_status.msg
    cmd_home_position.msg
    cmd_navigation_status.msg
    cmd_path_planning_mode.msg
    cmd_path_planning_status.msg
    cmd_sensor_status.msg
    cmd_vehicle_control_mode.msg
    cmd_vehicle_control_status.msg

    )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)


catkin_package(
        # INCLUDE_DIRS include
        #LIBRARIES sailboat_message
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        #DEPENDS system_lib
)


cmake_minimum_required(VERSION 2.8.3)
project(sailboat_message)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        FILES

        #driver
        Rudder_msg.msg
        Sail_msg.msg
        Mach_msg.msg
        GPS_msg.msg
        Ahrs_msg.msg
        WTST_msg.msg
        WTST_Pro_msg.msg
        Sensor_msg.msg
        Arduino_msg.msg
        Radar_msg.msg
        Pointinfo.msg

        Wind_Simulation_msg.msg
        Sailboat_Simulation_msg.msg
        Target_msg.msg

        station_keeping_out.msg
        fleet_race_out.msg
        scanning_out.msg
        avoidance_out.msg
        station_keeping_para.msg
        fleet_race_para.msg
        scanning_para.msg
        avoidance_para.msg

        traj_following_para.msg
        traj_following_out.msg

        Point.msg
        PointArray.msg

	obs_msg.msg
        obs_out.msg

        Out_Time_msg.msg
)

add_service_files(
  FILES
  Mach_srv.srv
  Control_srv.srv
  Self_Checking_srv.srv
  Out_Time_srv.srv
  Dxl_Control_srv.srv
  Dxl_State_srv.srv
#   Service1.srv
#   Service2.srv
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


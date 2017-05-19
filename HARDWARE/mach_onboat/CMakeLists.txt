cmake_minimum_required(VERSION 2.8.3)
project(mach_onboat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sailboat_message
        message_generation
  std_msgs
)

add_message_files(
        FILES
        Mach_msg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs

)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES mach_onboat
  CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

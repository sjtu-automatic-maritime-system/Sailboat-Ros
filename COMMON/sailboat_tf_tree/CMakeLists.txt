cmake_minimum_required(VERSION 2.8.3)
project(sailboat_tf_tree)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(sailboat_tf_tree_DEPENDENCE
        robot_state_publisher
        roscpp
        rospy
        std_msgs
        tf
        urdf
        xacro
        sailboat_message
#        turtlesim
        )

find_package(catkin REQUIRED COMPONENTS
    ${sailboat_tf_tree_DEPENDENCE}
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES sailboat_tf_tree
  CATKIN_DEPENDS ${sailboat_tf_tree_DEPENDENCE}
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})
add_dependencies(state_publisher ${catkin_EXPORTED_TARGETS})

add_executable(turtle_tf_broadcaster demo/tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener demo/tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

add_executable(frame_tf_broadcaster demo/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(sailboat_model)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #map_server
  sailboat_message
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
  turtlesim
  dynamic_reconfigure
  sailboat_message
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Sailboat_Mach_Simulation.cfg
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sailboat_model
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS})

set( sailboat_model_SRC
        lib/CCubicSplineInterpolation.cpp
        lib/CSailboatMotionEquation.cpp
        lib/CDynamicVPP.cpp
        )

add_library( ${PROJECT_NAME} ${sailboat_model_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )


add_executable(turtle_tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

add_executable(Sensor_Simulation_Pub src/Sensor_Simulation_Pub.cpp)
target_link_libraries(Sensor_Simulation_Pub ${catkin_LIBRARIES})

add_executable(Mach_Simulation_Pub src/Mach_Simulation_Pub.cpp)
add_dependencies(Mach_Simulation_Pub sailboat_model_gencfg)
target_link_libraries(Mach_Simulation_Pub ${catkin_LIBRARIES})

add_executable(Simulation src/Sailboat_Simulation.cpp)
target_link_libraries(Simulation ${catkin_LIBRARIES} sailboat_model)
add_dependencies(Simulation sailboat_message_generate_message_cpp)

add_executable(EigenTestDemo demo/EigenTestDemo.cpp)
target_link_libraries(EigenTestDemo ${catkin_LIBRARIES})

add_executable(SIPTestDemo demo/SplineInterpolationTestDemo.cpp)
target_link_libraries(SIPTestDemo ${catkin_LIBRARIES} sailboat_model)

add_executable(VppTestDemo demo/DynamicVppTestDemo.cpp)
target_link_libraries(VppTestDemo ${catkin_LIBRARIES} sailboat_model)

add_executable(MotionEquationTestDemo demo/MotionEquationTestDemo.cpp)
target_link_libraries(MotionEquationTestDemo ${catkin_LIBRARIES} sailboat_model)
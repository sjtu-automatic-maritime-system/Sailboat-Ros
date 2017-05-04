cmake_minimum_required(VERSION 2.8.3)
project(environment_simulation)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        sailboat_message
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


generate_dynamic_reconfigure_options(
        cfg/Sailboat_Mach_Simulation.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES environment_simulation
#  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS})



add_executable(Wind_Simulation_Pub src/Wind_Simulation_Pub.cpp)
target_link_libraries(Wind_Simulation_Pub ${catkin_LIBRARIES})

add_executable(Mach_Simulation_Pub src/Mach_Simulation_Pub.cpp)
add_dependencies(Mach_Simulation_Pub sailboat_model_gencfg)
target_link_libraries(Mach_Simulation_Pub ${catkin_LIBRARIES})


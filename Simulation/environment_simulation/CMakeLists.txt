cmake_minimum_required(VERSION 2.8.3)
project(environment_simulation)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        sailboat_message
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/Sailboat_Mach_Simulation.cfg
        cfg/Sailboat_targetAngle_Simulation.cfg
        cfg/wing_Simulation.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES environment_simulation
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs sailboat_message dynamic_reconfigure
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})



add_executable(wind_simulation_pub src/Wind_Simulation_Pub.cpp)
add_dependencies(wind_simulation_pub ${catkin_EXPORTED_TARGETS} environment_simulation_gencfg)
target_link_libraries(wind_simulation_pub ${catkin_LIBRARIES})

add_executable(mach_simulation_pub src/Mach_Simulation_Pub.cpp)
add_dependencies(mach_simulation_pub ${catkin_EXPORTED_TARGETS} environment_simulation_gencfg)
target_link_libraries(mach_simulation_pub ${catkin_LIBRARIES})

add_executable(targetangle_simulation_pub src/TargetAngle_Simulation_Pub.cpp)
add_dependencies(targetangle_simulation_pub environment_simulation_gencfg)
target_link_libraries(targetangle_simulation_pub ${catkin_LIBRARIES})

add_executable(wind_sensor_repub_simulation src/wind_sensor_repub_simulation.cpp)
target_link_libraries(wind_sensor_repub_simulation ${catkin_LIBRARIES})

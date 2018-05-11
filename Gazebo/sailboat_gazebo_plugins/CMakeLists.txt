cmake_minimum_required(VERSION 2.8.3)
project(sailboat_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  sailboat_message
  sailboat_simulation_lib
)


## Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Boost
#find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)


## Eigen
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories( include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        )
link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sailboat_message sailboat_simulation_lib
        DEPENDS #gazebo
        LIBRARIES
)

###########
## Build ##
###########
add_library(sailboat_gazebo_dynamics_plugin
        src/sailboat_gazebo_dynamics_plugin.cpp
        )

target_link_libraries(sailboat_gazebo_dynamics_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Eigen_LIBRARIES}
        )

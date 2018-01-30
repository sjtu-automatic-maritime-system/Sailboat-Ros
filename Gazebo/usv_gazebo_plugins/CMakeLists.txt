cmake_minimum_required(VERSION 2.8.3)
project(usv_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  roscpp 
  gazebo_ros
  nav_msgs
  geometry_msgs
  message_generation
)




## Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Boost
#find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)


## Eigen
find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#include_directories(${EIGEN_INCLUDE_DIRS}) 
#add_definitions(${EIGEN_DEFINITIONS})

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Drive.msg
)

generate_messages()

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


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp geometry_msgs nav_msgs  message_runtime
        DEPENDS gazebo
   LIBRARIES
)

###########
## Build ##
###########

## Declare a C++ library
add_library(usv_gazebo_dynamics_plugin
  src/usv_gazebo_dynamics_plugin.cpp
 )

add_dependencies(usv_gazebo_dynamics_plugin usv_gazebo_plugins_generate_messages_cpp)


target_link_libraries(usv_gazebo_dynamics_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
)


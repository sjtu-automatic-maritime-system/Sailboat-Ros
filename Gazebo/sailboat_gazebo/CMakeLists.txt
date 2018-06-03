cmake_minimum_required(VERSION 2.8.3)
project(sailboat_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(catkin_package_DEPENDENCIES
    gazebo_plugins
    gazebo_ros
    roscpp
    sensor_msgs
    tf
    hector_gazebo_plugins
        sailboat_message
        geometry_msgs
    )

find_package(catkin REQUIRED COMPONENTS
  ${catkin_package_DEPENDENCIES}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sailboat_gazebo
  CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(sensor_process_SRC
        src/sensor_simulation.cpp)


add_executable(sensor_simulation ${sensor_process_SRC})
target_link_libraries(sensor_simulation ${catkin_LIBRARIES})
add_dependencies(sensor_simulation ${catkin_EXPORTED_TARGETS})

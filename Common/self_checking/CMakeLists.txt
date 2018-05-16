cmake_minimum_required(VERSION 2.8.3)
project(self_checking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		roscpp
        rospy
        geometry_msgs
        sensor_msgs
        std_msgs
        dynamic_reconfigure
        sailboat_message
        dynamixel_workbench_msgs
        )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES self_checking
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs dynamic_reconfigure dynamixel_workbench_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
 ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}

 ${catkin_INCLUDE_DIRS}
)

set( checking_SRC
        src/main.cpp
        src/self_checking.cpp
        )

add_executable( ${PROJECT_NAME} ${checking_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})
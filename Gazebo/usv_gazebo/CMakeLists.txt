cmake_minimum_required(VERSION 2.8.3)
project(usv_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_package_DEPENDENCIES
	gazebo_ros
	roscpp
	sensor_msgs
    tf
	usv_gazebo_plugins
    sailboat_message
    )

find_package(catkin REQUIRED  COMPONENTS
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(usv_process_SRC
	src/send_drive_cmd.cpp)


add_executable(usv_drive_simulation ${usv_process_SRC})
target_link_libraries(usv_drive_simulation ${catkin_LIBRARIES})
add_dependencies(usv_drive_simulation ${catkin_EXPORTED_TARGETS})

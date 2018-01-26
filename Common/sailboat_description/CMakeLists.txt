cmake_minimum_required(VERSION 2.8.3)
project(sailboat_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(catkin_package_DEPENDENCIES
        ## set dependencies here
        roscpp
        rospy
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES})


catkin_package(
        #  INCLUDE_DIRS
        #  LIBRARIES gt_kf
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
        #  DEPENDS system_lib
)

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

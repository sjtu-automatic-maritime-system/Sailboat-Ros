cmake_minimum_required(VERSION 2.8.3)
project(object_detection_ros)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        object_detection_lib
        )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES}
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        #        INCLUDE_DIRS
        #  LIBRARIES gt_kf
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
        #  DEPENDS system_lib
)

include_directories(
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection src/detection_ros.cpp)
target_link_libraries(object_detection ${catkin_LIBRARIES} object_detection_lib)

add_executable(mytest src/test.cpp)
target_link_libraries(mytest ${catkin_LIBRARIES} object_detection_lib)

add_executable(img_undistortion src/undistort_image_raw.cpp)
target_link_libraries(img_undistortion ${catkin_LIBRARIES})
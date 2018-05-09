cmake_minimum_required(VERSION 2.8.3)
project(object_detection_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        cmake_modules
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        sailboat_message
        geometry_msgs
        )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES}
        )

find_package(OpenCV 3 REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("-- OpenCV version: ${OpenCV_VERSION}")
message("-- OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message("-- catkin include dirs: ${catkin_INCLUDE_DIRS}")
message("-- Eigen include dirs: ${Eigen_INCLUDE_DIRS}")


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        #        INCLUDE_DIRS
        #  LIBRARIES gt_kf
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
        #  DEPENDS system_lib
)

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)
set( detection_SRC
        src/detection_main.cpp
        src/detection_ros.cpp
        src/detection.cpp
        src/particle_filter.cpp
        #src/average.cpp
        )

add_executable(object_detection ${detection_SRC})
target_link_libraries(object_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})

# set( test_SRC
#         test/test.cpp
#         src/detection.cpp
#         )

#add_executable(mytest ${test_SRC})
#add_dependencies(mytest ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mytest ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(img_undistortion src/undistort_image_raw.cpp)
#target_link_libraries(img_undistortion ${catkin_LIBRARIES})

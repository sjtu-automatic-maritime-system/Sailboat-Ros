cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_lib)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set( catkin_package_DEPENDENCIES
    ## set dependencies here
)

find_package( catkin REQUIRED COMPONENTS ${catkin_package_DEPENDENCIES} )

catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
)

###########
## Build ##
###########
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

## Create library
set( sensor_fusion_lib_SRC
    src/kf_2d.cpp
    src/tracking.cpp
)
add_library( ${PROJECT_NAME} ${sensor_fusion_lib_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

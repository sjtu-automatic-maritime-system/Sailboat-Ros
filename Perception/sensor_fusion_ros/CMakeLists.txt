cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_ros)


## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        roscpp
        std_msgs
        sensor_fusion_msg
        sensor_fusion_lib
	sailboat_message
        )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES}
        )



catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES gt_kf
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
        #  DEPENDS system_lib
)


include_directories(
        #     include
        ${catkin_INCLUDE_DIRS}
)



add_executable(gps_kf_node gps_kf/gps_kf_node.cpp)
target_link_libraries(gps_kf_node ${catkin_LIBRARIES})
add_executable(kf_test_node unittest/test_node.cpp)
target_link_libraries(kf_test_node ${catkin_LIBRARIES})


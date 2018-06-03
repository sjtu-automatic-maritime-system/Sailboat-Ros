cmake_minimum_required(VERSION 2.8.3)
project(traj_following)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        dynamic_reconfigure
        roscpp
        rospy
        sailboat_message
        std_msgs
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES}
        )


generate_dynamic_reconfigure_options(
       cfg/traj_following.cfg
#        #   cfg/DynReconf1.cfg
#        #   cfg/DynReconf2.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES traj_following
        CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sailboat_message std_msgs
        #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})


#add_executable(astar_test src/Astar.cpp src/main_test.cpp)
#target_link_libraries(astar_test ${catkin_LIBRARIES} sensor_fusion_lib)

set( traj_following_SRC
        src/ert_main.cpp
        src/path_following.cpp
        src/rt_nonfinite.cpp
        src/rtGetInf.cpp
        src/rtGetNaN.cpp
        )

add_executable(traj_following_ros ${traj_following_SRC})
add_dependencies(traj_following_ros ${catkin_EXPORTED_TARGETS} traj_following_gencfg)
target_link_libraries(traj_following_ros ${catkin_LIBRARIES})

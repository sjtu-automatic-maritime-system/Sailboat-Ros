cmake_minimum_required(VERSION 2.8.3)
project(path_planning_astar)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# # check c++11 / c++0x
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if (COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "-std=c++11")
# elseif (COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "-std=c++0x")
# else ()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif ()


## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        dynamic_reconfigure
        roscpp
        std_msgs
        sensor_fusion_lib
	sailboat_message
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_package_DEPENDENCIES}
        )


generate_dynamic_reconfigure_options(
        cfg/path_planning_Config.cfg
        #   cfg/DynReconf1.cfg
        #   cfg/DynReconf2.cfg
)


catkin_package(
        #  INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
        #  DEPENDS system_lib
)


include_directories(
        include/astar
        ${catkin_INCLUDE_DIRS}
)


#add_executable(astar_test src/Astar.cpp src/main_test.cpp)
#target_link_libraries(astar_test ${catkin_LIBRARIES} sensor_fusion_lib)

add_executable(astar_ros_1020 src/Astar.cpp src/main_ros_1020.cpp)
target_link_libraries(astar_ros_1020 ${catkin_LIBRARIES})

add_executable(astar_ros_1027 src/Astar.cpp src/main_ros_1027.cpp)
target_link_libraries(astar_ros_1027 ${catkin_LIBRARIES})

add_executable(astar_ros_1028_sm src/Astar.cpp src/main_ros_1028_simulation.cpp)
target_link_libraries(astar_ros_1028_sm ${catkin_LIBRARIES})

add_executable(astar_ros_1028 src/Astar.cpp src/main_ros_1028.cpp)
target_link_libraries(astar_ros_1028 ${catkin_LIBRARIES})

add_executable(astar_ros src/Astar.cpp src/main_ros.cpp)
add_dependencies(astar_ros ${catkin_EXPORTED_TARGETS} path_planning_astar_gencfg)
target_link_libraries(astar_ros ${catkin_LIBRARIES})

add_executable(astar_simulation_ros src/Astar.cpp src/main_simulation_ros.cpp)
target_link_libraries(astar_simulation_ros ${catkin_LIBRARIES})

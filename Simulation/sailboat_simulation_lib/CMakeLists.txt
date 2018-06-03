cmake_minimum_required(VERSION 2.8.3)
project(sailboat_simulation_lib)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  math_tool_lib
  roscpp
  rospy
  sailboat_message
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sailboat_simulation_lib
  CATKIN_DEPENDS math_tool_lib roscpp rospy sailboat_message std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS})

set( sailboat_simulation_lib_SRC
        #lib/CCubicSplineInterpolation.cpp
        src/CSimulationVer1.cpp
        src/CSimulationVer2.cpp
        #src/Sailboat_Simulation.cpp
        )

add_library( ${PROJECT_NAME} ${sailboat_simulation_lib_SRC} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(eigen_test_demo demo/EigenTestDemo.cpp)
target_link_libraries(eigen_test_demo ${catkin_LIBRARIES})
#
add_executable(motion_equation_test_demo demo/MotionEquationTestDemo.cpp)
target_link_libraries(motion_equation_test_demo ${catkin_LIBRARIES} sailboat_simulation_lib)

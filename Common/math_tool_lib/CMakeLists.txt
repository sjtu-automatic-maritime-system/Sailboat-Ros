cmake_minimum_required(VERSION 2.8.3)
project(math_tool_lib)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sailboat_message
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES math_tool_lib
  CATKIN_DEPENDS roscpp rospy sailboat_message std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

set( math_tool_lib_SRC
        src/CCubicSplineInterpolation.cpp
        src/CPID.cpp)


add_library( ${PROJECT_NAME} ${math_tool_lib_SRC} )
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(sip_test_demo demo/SplineInterpolationTestDemo.cpp)
target_link_libraries(sip_test_demo ${catkin_LIBRARIES} math_tool_lib)

add_executable(pid_demo demo/pidDemo.cpp)
target_link_libraries(pid_demo ${catkin_LIBRARIES} math_tool_lib)

add_executable(math_demo demo/MathDemo.cpp)
target_link_libraries(math_demo ${catkin_LIBRARIES} math_tool_lib)
cmake_minimum_required(VERSION 2.8.3)
project(object_detection_lib)


########################
##   Catkin Package   ##
########################

## This is a 'lib' package
## Thus, it is just a dummy catkin package to provide headers and libraries for other packages
## There should be no code which depends on ROS in any way!

## Catkin packages required for both building and running (<depend> entries in package.xml)
set(catkin_package_DEPENDENCIES
        ## set dependencies here
        )

## Find catkin macros and libraries
## If COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
## All build dependencies have to appear here (<depend>, <build_depend> and <build_export_depend> entries in package.xml)
find_package(catkin REQUIRED COMPONENTS ${catkin_package_DEPENDENCIES})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
## LIBRARIES - The exported libraries from the project
## CATKIN_DEPENDS - Other catkin projects that this project depends on (runtime dependencies: <depend> and <exec_depend> entries in package.xml)
## DEPENDS - Non-catkin CMake projects that this project depends on
catkin_package(
        INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_package_DEPENDENCIES}
)

###########
## Build ##
###########

## Cxx compiler flags
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV 2.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("-- OpenCV version: ${OpenCV_VERSION}")
message("-- OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message("-- catkin include dirs: ${catkin_INCLUDE_DIRS}")

## Create library
set(object_detection_lib_SRC
        src/detection.cpp
        )
add_library(${PROJECT_NAME} ${object_detection_lib_SRC})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})


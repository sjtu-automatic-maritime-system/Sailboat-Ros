cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_lib)


########################
##   Catkin Package   ##
########################

## This is a 'lib' package
## Thus, it is just a dummy catkin package to provide headers and libraries for other packages
## There should be no code which depends on ROS in any way!

## Catkin packages required for both building and running (<depend> entries in package.xml)
set( catkin_package_DEPENDENCIES
    ## set dependencies here
)

## Find catkin macros and libraries
## If COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
## All build dependencies have to appear here (<depend>, <build_depend> and <build_export_depend> entries in package.xml)
find_package( catkin REQUIRED COMPONENTS ${catkin_package_DEPENDENCIES} )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gt_kf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

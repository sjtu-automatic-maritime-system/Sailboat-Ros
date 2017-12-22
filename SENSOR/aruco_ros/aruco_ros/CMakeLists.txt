cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  tf
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
  mavros_msgs
        nodelet
)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros
  CATKIN_DEPENDS aruco sensor_msgs tf nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nodelet_aruco_single src/nodelets.cpp
        src/simple_single.cpp
        src/aruco_ros_utils.cpp)
target_link_libraries(nodelet_aruco_single ${catkin_LIBRARIES})

# add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
# target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES})

add_executable(aruco_single src/single_aruco_node.cpp
                      src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(aruco_single ${PROJECT_NAME}_gencfg)
target_link_libraries(aruco_single ${catkin_LIBRARIES})

# add_executable(double src/simple_double.cpp
#                       src/aruco_ros_utils.cpp)
# add_dependencies(double ${PROJECT_NAME}_gencfg)
# target_link_libraries(double ${catkin_LIBRARIES})

# add_executable(marker_publisher src/marker_publish.cpp
#                                 src/aruco_ros_utils.cpp)
# add_dependencies(marker_publisher ${PROJECT_NAME}_gencfg)
# target_link_libraries(marker_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco_single nodelet_aruco_single
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


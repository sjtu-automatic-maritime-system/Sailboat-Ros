#include <ros/ros.h>
#include "aruco_ros/simple_single.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_ros");

    aruco_ros::ArucoSimple node(ros::NodeHandle(),ros::NodeHandle("~"));

    ros::spin();
}

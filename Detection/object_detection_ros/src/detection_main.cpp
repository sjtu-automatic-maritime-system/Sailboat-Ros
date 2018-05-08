#include "object_detection_ros/detection_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");

    //DetectionRos node(ros::NodeHandle());
    DetectionRos node;

    ros::spin();
    return 0;
}

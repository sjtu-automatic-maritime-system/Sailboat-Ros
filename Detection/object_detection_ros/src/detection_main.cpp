#include "object_detection_ros/detection_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");

    //DetectionRos node(ros::NodeHandle());
    DetectionRos node(0.5,80);

    ros::spin();
    return 0;
}

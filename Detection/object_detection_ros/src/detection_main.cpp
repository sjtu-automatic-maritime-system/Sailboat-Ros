#include "object_detection_ros/detection_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");

    //DetectionRos node(ros::NodeHandle());
    //DetectionRos node(0.5,80,false);
    DetectionRos node(0.5,60,false,false);
    ros::spin();
    return 0;
}

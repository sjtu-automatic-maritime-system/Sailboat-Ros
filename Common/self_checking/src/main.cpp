#include "self_checking.h"

// using namespace rosmsg;
// using namespace cu;
// using namespace wapp;
int main(int argc, char **argv) {
    ros::init(argc, argv, "self_checking");
    SailboatSelfChecking::getInstance().onInit();
    ROS_INFO("ros run!");
    // ROS stuff
    ros::spin();
    return 0;
}
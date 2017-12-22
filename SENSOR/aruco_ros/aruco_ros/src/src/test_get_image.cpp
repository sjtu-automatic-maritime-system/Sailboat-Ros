
#include <ros/ros.h>
#include <image_transport/image_transport.h>


namespace aruco_ros {
class ArucoSimple {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
public:
ArucoSimple(ros::NodeHandle _comm_nh, ros::NodeHandle _private_nh)
        : nh(_comm_nh),
          it(_comm_nh) {
                image_sub = it.subscribe("image_raw", 1, &ArucoSimple::image_callback,this);
          }

void image_callback(const sensor_msgs::ImageConstPtr &msg) {
    // if ((image_pub.getNumSubscribers() == 0) &&
    //     (debug_pub.getNumSubscribers() == 0) {
    //     ROS_DEBUG("No subscribers, not looking for aruco markers");
    //     return;
    // }

    ROS_INFO("get image");
}

};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    /* code for main function */
    aruco_ros::ArucoSimple test(ros::NodeHandle(),ros::NodeHandle("~"));
    
    ros::spin();
    return 0;
}

 


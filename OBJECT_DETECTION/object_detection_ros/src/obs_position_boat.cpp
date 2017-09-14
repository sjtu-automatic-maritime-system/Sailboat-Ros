#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tld_msgs/BoundingBox.h"
#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"


#define IMG_WIDTH 1296
#define IMG_HEIGHT 964
#define FOV 100 //field of view 100 deg
#define DISTANCE_TO_BOW 0.2 // m
#define PIXELS_TO_BOW 50 // pixels

using namespace std;
//using namespace cv;


void callback(const tld_msgs::BoundingBoxConstPtr &bbox_msg) {
    // x, y bbox upper left corner coord
    int x = bbox_msg->x;
    int y = bbox_msg->y;
    int height = bbox_msg->height;
    int width = bbox_msg->width;
    // x_center y_center center point of bbox
    int x_center = x + width / 2;
    int y_center = y + height / 2;



}


int main(int argc, char **argv) {
    ros::init(argc, argv, "undistort_image");
    ros::NodeHandle nh;

//    ros::Publisher sensor_pub = nh.advertise<sailboat_message::Sensor_msg>("sensor", 2);
    ros::Subscriber bbox_sub = nh.subscribe("/tld_tracked_object", 2, &callback);

    ros::spin();
    return 0;
}

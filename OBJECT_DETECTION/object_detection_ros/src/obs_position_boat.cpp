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


void callback(const sensor_msgs::ImageConstPtr &img_in) {
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in, "bgr8")->image.copyTo(src_img);

    cv::Mat img_undistorted;
    cv::undistort(src_img, img_undistorted, cameraMatrix, distCoeffs);

    sensor_msgs::ImagePtr img_undistorted_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                                   img_undistorted).toImageMsg();
    pub_img_undistorted.publish(img_undistorted_msg);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "undistort_image");
    ros::NodeHandle nh;

    ros::Publisher sensor_pub = nh.advertise<sailboat_message::Sensor_msg>("sensor", 2);
    message_filters::Subscriber<tld_msgs::BoundingBox> bbox_sub(nh, "/tld_tracked_object", 2);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/image_undistorted", 2);
    message_filters::S

    typedef sync_policies::ApproximateTime<sailboat_message::Ahrs_msg, sailboat_message::WTST_msg> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ahrs_sub, wtst_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}

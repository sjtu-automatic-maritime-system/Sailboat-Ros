#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detection_lib/detection.h"

using namespace std;
using namespace cv;

image_transport::Publisher  pub_img_edge;
image_transport::Publisher  pub_img_dst;

void detection_cb(const sensor_msgs::Image::ConstPtr& img_in)
{
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in,"bgr8")->image.copyTo(src_img);

    cv::Mat edge = detection::edgeDetection(src_img, 150, 200);
    std::vector<cv::Vec3f> circles = detection::circleDectection(src_img, edge);
    std::cout << "detected circles: " << circles.size() << std::endl;
    for (size_t i = 0; i < circles.size(); i++) {
        std::cout << "circle " << i << circles[i] << std::endl;
    }
    sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edge).toImageMsg();
    pub_img_edge.publish(edge_msg);

    sensor_msgs::ImagePtr dst_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img).toImageMsg();
    pub_img_dst.publish(dst_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;

    ros::Subscriber sub_image =  nh.subscribe("camera/image_rect_color", 2, &detection_cb);

    image_transport::ImageTransport it(nh);
    pub_img_edge = it.advertise("hue_edges", 2);
    pub_img_dst = it.advertise("dst_circles", 2);

    ros::spin();
    return 0;
}

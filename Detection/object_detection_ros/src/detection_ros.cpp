#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "object_detection_ros/detection.h"


#define IMG_WIDTH 1296
#define IMG_HEIGHT 964

using namespace std;
//using namespace cv;

image_transport::Publisher  pub_img_edge;
image_transport::Publisher  pub_img_dst;

void detection_cb(const sensor_msgs::ImageConstPtr& img_in)
{
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in,"bgr8")->image.copyTo(src_img);
//    src_img = cv::imdecode(cv::Mat(img_in->data),3);//convert compressed image data to cv::Mat
//    cv::imshow("src", src_img);
//    cv::waitKey(5);
    cv::Mat src_ROI;
    int src_ROI_p1_y = IMG_HEIGHT/3;
    src_ROI = src_img(cv::Rect(0,src_ROI_p1_y,IMG_WIDTH,IMG_HEIGHT-src_ROI_p1_y));
    cv::Mat edge = detection::edgeDetection(src_ROI, 150, 200);
    std::vector<cv::Vec3f> circles = detection::circleDectection(src_ROI, edge);
    std::cout << "detected circles: " << circles.size() << std::endl;
    for (size_t i = 0; i < circles.size(); i++) {
        std::cout << "circle " << i << circles[i] << std::endl;
    }

//    cv::imshow("src", src_ROI);
//    cv::waitKey(5);

    sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edge).toImageMsg();
    pub_img_edge.publish(edge_msg);

    sensor_msgs::ImagePtr dst_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_ROI).toImageMsg();
    pub_img_dst.publish(dst_msg);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;

//    ros::Subscriber sub_image =  nh.subscribe("camera/image_rect_color", 2, &detection_cb);
    ros::Subscriber sub_image =  nh.subscribe("camera/image_raw", 2, &detection_cb);
//    ros::Subscriber sub_image =  nh.subscribe("camera/image_raw/compressed", 2, &detection_cb);

    image_transport::ImageTransport it(nh);
    pub_img_edge = it.advertise("edges", 2);
    pub_img_dst = it.advertise("dst_circles", 2);

    ros::spin();
    return 0;
}

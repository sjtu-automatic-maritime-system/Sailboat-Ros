#include "ros/ros.h"

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

#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"


using namespace std;
//using namespace cv;

image_transport::Publisher pub_img_undistorted;
image_transport::Publisher pub_img_undistorted_rotated;

cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type); // Intrisic matrix;
cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector;

void get_camera_info() {
//  camera info from file ../camera_info/0.yaml
//  camera matrix: [816.118268598647, 0, 680.6511245884145, 0, 822.0196620588329, 458.230641061779, 0, 0, 1]
    cameraMatrix.at<double>(0, 0) = 816.118268598647;
    cameraMatrix.at<double>(0, 1) = 0.000000;
    cameraMatrix.at<double>(0, 2) = 680.6511245884145;

    cameraMatrix.at<double>(1, 0) = 0.000000;
    cameraMatrix.at<double>(1, 1) = 822.0196620588329;
    cameraMatrix.at<double>(1, 2) = 458.230641061779;

    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1.000;

//  distortion coeffs: [-0.3014687793515643, 0.08833386719988248, -0.0008052827098236777, -0.004742307626243158, 0]
    distCoeffs.at<double>(0) = -0.3014687793515643;
    distCoeffs.at<double>(1) = 0.08833386719988248;
    distCoeffs.at<double>(2) = -0.0008052827098236777;
    distCoeffs.at<double>(3) = -0.004742307626243158;
    distCoeffs.at<double>(4) = 0.0;
}

void callback(const sailboat_message::Ahrs_msgConstPtr &ahrs_msg, const sensor_msgs::ImageConstPtr &img_in) {
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in, "bgr8")->image.copyTo(src_img);

    cv::Mat img_undistorted;
    cv::undistort(src_img, img_undistorted, cameraMatrix, distCoeffs);

    sensor_msgs::ImagePtr img_undistorted_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                                   img_undistorted).toImageMsg();
    pub_img_undistorted.publish(img_undistorted_msg);

    if (1) {
        double roll = ahrs_msg->roll * 57.3; //deg
        cv::Point2f center = cv::Point2f(src_img.cols / 2, src_img.rows / 2);  // 旋转中心

        cv::Mat rotateMat;
        rotateMat = cv::getRotationMatrix2D(center, roll, 1);

        cv::Mat rotateImg;
        cv::warpAffine(src_img, rotateImg, rotateMat, src_img.size());

        sensor_msgs::ImagePtr img_undistorted_rotated_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                                               rotateImg).toImageMsg();
        pub_img_undistorted_rotated.publish(img_undistorted_rotated_msg);
    }

}


int main(int argc, char **argv) {
    get_camera_info();
    ros::init(argc, argv, "undistort_image");
    ros::NodeHandle nh;

    message_filters::Subscriber<sailboat_message::Ahrs_msg> ahrs_sub(nh, "/ahrs", 2);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/image_raw", 2);

    typedef message_filters::sync_policies::ApproximateTime<sailboat_message::Ahrs_msg, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), ahrs_sub, img_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    image_transport::ImageTransport it(nh);
    pub_img_undistorted = it.advertise("/camera/image_undistorted", 2);
    pub_img_undistorted_rotated = it.advertise("/camera/image_undistorted_rotated", 2);

    ros::spin();
    return 0;
}

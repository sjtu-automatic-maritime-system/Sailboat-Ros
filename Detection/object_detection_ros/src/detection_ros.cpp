#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sailboat_message/Sensor_msg.h"
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "object_detection_ros/detection.h"
#include "cmath"

#define IMG_WIDTH 1296
#define IMG_HEIGHT 964

//#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Dense>

//ball x 10 y 20
//     x 25.5 y -4.1


using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
// using Eigen::MatrixXd;
using namespace std;

image_transport::Publisher  pub_img_edge;
image_transport::Publisher  pub_img_dst;

double posX = 0;
double posY = 0;
double pitch = 0;
double roll = 0;
double yaw = 0;
double f = IMG_WIDTH/(2*std::tan(40/57.3));

//roll pitch yaw
void roll_pitch_yaw_to_R(Vector3d E,Matrix3d &R){
    double sx = sin(E(0));
    double cx = cos(E(0));
    double sy = sin(E(1));
    double cy = cos(E(1));
    double sz = sin(E(2));
    double cz = cos(E(2));
    R(0,0) = cy*cz;
    R(0,1) = cz*sx*sy-cx*sz;
    R(0,2) = sx*sz+cx*cz*sy;
    R(1,0) = cy*sz;
    R(1,1) = cx*cz+sx*sy*sz;
    R(1,2) = cx*sy*sz-cz*sz;
    R(2,0) = -sy;
    R(2,1) = cy*sx;
    R(2,2) = cx*cy;
    cout<<"R=\n"<<R<<endl;
}


void detection_cb(const sensor_msgs::ImageConstPtr& img_in)
{
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in,"bgr8")->image.copyTo(src_img);
//    src_img = cv::imdecode(cv::Mat(img_in->data),3);//convert compressed image data to cv::Mat
//    cv::imshow("src", src_img);
//    cv::waitKey(5);
    cv::Mat src_ROI;
    //int src_ROI_p1_y = IMG_HEIGHT/3;
    int src_ROI_p1_y = 0;
    src_ROI = src_img(cv::Rect(0,src_ROI_p1_y,IMG_WIDTH,IMG_HEIGHT-src_ROI_p1_y));
    cv::Mat edge = detection::edgeDetection(src_ROI, 150, 200);
    std::vector<cv::Vec3f> circles = detection::circleDectection(src_ROI, edge);
    std::cout << "detected circles: " << circles.size() << std::endl;
    //for (size_t i = 0; i < circles.size(); i++) {
    //    std::cout << "circle " << i+1 << " = " <<circles[i] << std::endl;
    if (circles.size()>0){
        
        double h_angle = std::atan((circles[0][0]-IMG_WIDTH/2)/f);
        double v_angle = std::atan((circles[0][1]-IMG_HEIGHT/2)/f);
        //std::cout << "h_angle = " << h_angle*57.3 << std::endl;
        //std::cout << "v_angle = " << v_angle*57.3 << std::endl;
        //double distance = std::sqrt(std::pow((posX-25.5),2.0)+std::pow((posY+4.1),2.0));
        double distance_cal = 0.5/circles[0][2]*std::sqrt(std::pow(circles[0][0]-IMG_WIDTH/2,2)+std::pow(circles[0][1]-IMG_HEIGHT/2,2)+std::pow(f,2));
        //std::cout << "distance = " << distance << std::endl;
        std::cout << "distance_cal = " << distance_cal << std::endl;

        Matrix3d R_tmp;
        //Matrix3d R_in_tmp;
        Vector3d E_tmp;
        R_tmp = Matrix3d::Zero(3,3);
        E_tmp(0) = roll;
        E_tmp(1) = pitch;
        E_tmp(2) = 0;
        roll_pitch_yaw_to_R(E_tmp,R_tmp);
        R_tmp = R_tmp.inverse().eval();

        MatrixXd L_ship(3,1);
        MatrixXd L_world(3,1);
        L_ship(0,0) = f;
        L_ship(1,0) = circles[0][0]-IMG_WIDTH/2;
        L_ship(2,0) = circles[0][1]-IMG_HEIGHT/2;
        L_world = R_tmp*L_ship;
        //std::cout << "L_ship : " << L_ship << std::endl;
        //std::cout << "L_world : " << L_world << std::endl;
        
        double h_angle_final = std::atan((L_world(1,0))/f);
        double v_angle_final = std::atan((L_world(2,0))/f);
        //std::cout << "h_angle = " << h_angle*57.3 << std::endl;
        //std::cout << "v_angle = " << v_angle*57.3 << std::endl;

        double object_yaw = yaw + h_angle_final;

        double object_x = posX + distance_cal*cos(object_yaw);
        double object_y = posY + distance_cal*sin(object_yaw);

        ROS_INFO("object pos : ( %f , %f )",object_x, object_y);
    }

    sensor_msgs::ImagePtr edge_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edge).toImageMsg();
    pub_img_edge.publish(edge_msg);

    sensor_msgs::ImagePtr dst_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_ROI).toImageMsg();
    pub_img_dst.publish(dst_msg);

}

void sensor_cb(const sailboat_message::Sensor_msg::ConstPtr msg){
    //ROS_INFO("get sensor");
    posX = msg->Posx;
    posY = msg->Posy;
    roll = msg->Roll;
    pitch = msg->Pitch;
    yaw = msg->Yaw;
    double distance = std::sqrt(std::pow((posX-25.5),2.0)+std::pow((posY+4.1),2.0));
    //std::cout << "sensor distance = " << distance << "; posX = "<<posX<<"; posY = "<<posY<< std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;

//    ros::Subscriber sub_image =  nh.subscribe("camera/image_rect_color", 2, &detection_cb);
    ros::Subscriber sub_image =  nh.subscribe("/usv/camera1/image_raw", 2, &detection_cb);
    ros::Subscriber gps =  nh.subscribe("/sensor", 2, &sensor_cb);
//    ros::Subscriber sub_image =  nh.subscribe("camera/image_raw/compressed", 2, &detection_cb);

    image_transport::ImageTransport it(nh);
    pub_img_edge = it.advertise("edges", 2);
    pub_img_dst = it.advertise("dst_circles", 2);

    ros::spin();
    return 0;
}

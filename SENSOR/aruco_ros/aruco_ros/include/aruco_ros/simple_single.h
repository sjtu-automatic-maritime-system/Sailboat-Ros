//
// Created by hywel on 17-12-20.
//

#ifndef ARUCO_ROS_SIMPLE_SINGLE_H_H
#define ARUCO_ROS_SIMPLE_SINGLE_H_H

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <queue>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <mavros_msgs/LandingTarget.h>

#define pi 3.1415926

using namespace aruco;

namespace aruco_ros {
class ArucoSimple {
private:
    cv::Mat inImage;
    aruco::CameraParameters camParam;

    bool useRectifiedImages;
    MarkerDetector mDetector;
    vector<Marker> markers;
    ros::Subscriber cam_info_sub;
    bool cam_info_received;
    image_transport::Publisher image_pub;
    image_transport::Publisher debug_pub;

    std::string camera_info_path;
    std::string image_path;

    bool isSmallestAruco;

    bool isPubDebug;

    ros::ServiceClient client;

    double marker_size;
    int marker_id;
    uint32_t active_marker;

    map<uint32_t, float> markerSizes;
    map<uint32_t, queue<int> > marker_history_queue;
    //map<uint32_t,MarkerPoseTracker> MTracker;

    int marker_history;
    int marker_threshold;

    bool output;
    bool verbose;

    double fovx, fovy;
    int inputwidth, inputheight;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
    ArucoSimple(ros::NodeHandle _comm_nh, ros::NodeHandle _private_nh);

    void image_callback(const sensor_msgs::ImageConstPtr &msg);

    void cam_info_callback(const sensor_msgs::CameraInfo &msg);

    void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level);

    void drawARLandingCube(cv::Mat &Image, Marker &m, const CameraParameters &CP);

    void drawVectors(cv::Mat &in, cv::Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y,
                     double Distance, double cX, double cY);

    int markerHistory(map<uint32_t, queue<int> > &marker_history_queue, uint32_t thisId, uint32_t marker_history);

    void changeActiveMarker(map<uint32_t, queue<int> > &marker_history_queue, uint32_t &active_marker, uint32_t newId,
                            uint32_t marker_history);

};
};


#endif //ARUCO_ROS_SIMPLE_SINGLE_H_H

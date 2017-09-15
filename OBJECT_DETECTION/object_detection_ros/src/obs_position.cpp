#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>

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
#include "geometry_msgs/PointStamped.h"

#define IMG_WIDTH 1296
#define IMG_HEIGHT 964
#define FOV 100 //field of view 100 deg
#define DISTANCE_TO_BOW 0.2 // m
#define PIXELS_TO_BOW 50 // pixels

using namespace std;
//using namespace cv;
using Eigen::VectorXd;
using Eigen::MatrixXd;

ros::Publisher obs_boat_pub;
ros::Publisher obs_ground_pub;
Eigen::Matrix4d T_boat_to_ground;

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
    return rz * ry * rx;
}


//
void bbox_cb(const tld_msgs::BoundingBoxConstPtr &bbox_msg) {
    // x, y bbox upper left corner coord
    int x = bbox_msg->x;
    int y = bbox_msg->y;
    int height = bbox_msg->height;
    int width = bbox_msg->width;
    // x_center y_center center point of bbox
    int x_center = x + width / 2;
    int y_center = y + height / 2;

    double direction = (x_center - IMG_HEIGHT / 2) / (double) IMG_WIDTH * FOV;
    double distance = (IMG_HEIGHT - y_center) / (double) PIXELS_TO_BOW * DISTANCE_TO_BOW;

    geometry_msgs::PointStamped obs_pos;
    obs_pos.header = bbox_msg->header;
    obs_pos.header.frame_id = "wtst";
    obs_pos.point.x = distance * cos(direction / 57.3);
    obs_pos.point.y = distance * sin(direction / 57.3);
    obs_pos.point.z = 0;
    std::cout << "obs_pos x: " << obs_pos.point.x << "  y: " << obs_pos.point.y << endl;
    obs_boat_pub.publish(obs_pos);


    VectorXd h_obs_boat(4, 1);
    h_obs_boat << obs_pos.point.x, obs_pos.point.y, 0, 1;
    VectorXd h_obs_ground = T_boat_to_ground * h_obs_boat;
    geometry_msgs::PointStamped obs_pos_g;
    obs_pos_g.header = bbox_msg->header;
    obs_pos_g.header.frame_id = "map";
    obs_pos_g.point.x = h_obs_ground[0];
    obs_pos_g.point.y = h_obs_ground[1];
    obs_pos_g.point.z = h_obs_ground[2];
    std::cout << "obs_pos_ground x: " << obs_pos_g.point.x << "  y: " << obs_pos_g.point.y << endl;
    obs_ground_pub.publish(obs_pos_g);
}


void wtst_cb(const sailboat_message::WTST_msgConstPtr &wtst_msg) {
//    std::cout << "wtst yaw: " << wtst_msg->Yaw << std::endl;
    Eigen::Affine3d r = create_rotation_matrix(0, 0, wtst_msg->Yaw / 57.3);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(wtst_msg->PosX, wtst_msg->PosY, 0)));
    T_boat_to_ground = (t * r).matrix();

}


//void callback(const sailboat_message::WTST_msgConstPtr &wtst_msg,
//              const tld_msgs::BoundingBoxConstPtr &bbox_msg) {
//    // x, y bbox upper left corner coord
//    int x = bbox_msg->x;
//    int y = bbox_msg->y;
//    int height = bbox_msg->height;
//    int width = bbox_msg->width;
//    // x_center y_center center point of bbox
//    int x_center = x + width / 2;
//    int y_center = y + height / 2;
//
//    double direction = (x_center - IMG_HEIGHT / 2) / (double) IMG_WIDTH * FOV;
//    double distance = (IMG_HEIGHT - y_center) / (double) PIXELS_TO_BOW * DISTANCE_TO_BOW;
//
//    geometry_msgs::PointStamped obs_pos;
//    obs_pos.header = bbox_msg->header;
//    obs_pos.header.frame_id = "WTST";
//    obs_pos.point.x = distance * cos(direction / 57.3);
//    obs_pos.point.y = distance * sin(direction / 57.3);
//    obs_pos.point.z = 0;
//    std::cout << "obs_pos x: " << obs_pos.point.x << "  y: " << obs_pos.point.y << endl;
//    obs_boat_pub.publish(obs_pos);
//
//    std::cout << "wtst yaw: " << wtst_msg->Yaw << std::endl;
//    Eigen::Affine3d r = create_rotation_matrix(0, 0, wtst_msg->Yaw);
//    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(wtst_msg->PosX, wtst_msg->PosY, 0)));
//    Eigen::Matrix4d T_boat_to_ground = (t * r).matrix();
//    VectorXd h_obs_boat(4, 1);
//    h_obs_boat << obs_pos.point.x, obs_pos.point.y, 0, 1;
//    VectorXd h_obs_ground = T_boat_to_ground * h_obs_boat;
//
//    geometry_msgs::PointStamped obs_pos_g;
//    obs_pos_g.header = bbox_msg->header;
//    obs_pos_g.header.frame_id = "WTST";
//    obs_pos_g.point.x = h_obs_ground[0];
//    obs_pos_g.point.y = h_obs_ground[1];
//    obs_pos_g.point.z = h_obs_ground[2];
//    std::cout << "obs_pos_ground x: " << obs_pos_g.point.x << "  y: " << obs_pos_g.point.y << endl;
//    obs_ground_pub.publish(obs_pos_g);
//
//
//}


int main(int argc, char **argv) {
    ros::init(argc, argv, "obs_position");
    ros::NodeHandle nh;

    obs_boat_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_boat_position", 2);
    obs_ground_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_ground_position", 2);

    ros::Subscriber bbox_sub = nh.subscribe("/tld_tracked_object", 2, &bbox_cb);
    ros::Subscriber wtst_sub = nh.subscribe("/wtst", 2, &wtst_cb);


//    message_filters::Subscriber<sailboat_message::WTST_msg> wtst_sub(nh, "/wtst", 2);
//    message_filters::Subscriber<tld_msgs::BoundingBox> bbox_sub(nh, "/tld_tracked_object", 2);
//
//    typedef message_filters::sync_policies::ApproximateTime<sailboat_message::WTST_msg, tld_msgs::BoundingBox> MySyncPolicy;
//    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), wtst_sub, bbox_sub);
//    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();
    return 0;
}

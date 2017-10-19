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
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "tld_msgs/BoundingBox.h"
#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "geometry_msgs/PointStamped.h"
#include "sailboat_message/Detected_obs.h"

#include "sensor_fusion_lib/measurement_package.h"
#include "sensor_fusion_lib/tracking.h"
#include "sensor_fusion_msg/GpsKF.h"


#define BOAT_LENGTH 1.5  // m


using namespace std;
//using namespace cv;
using Eigen::VectorXd;
using Eigen::MatrixXd;

image_transport::Publisher pub_img;

ros::Publisher obs_boat_pub;
ros::Publisher obs_ground_pub;
ros::Publisher obs_ground_filter_pub;

Eigen::Matrix4d T_boat_to_ground;

Tracking tracking;


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
    return rz * ry * rx;
}


void obs_cb(const sailboat_message::Detected_obsConstPtr &obs_msg) {
    if (obs_msg->confidence > 0) {
        double depth = obs_msg->depth;
        double distance = obs_msg->distance;
        double direction = obs_msg->direction;


        geometry_msgs::PointStamped obs_pos;
        obs_pos.header = obs_msg->header;
        obs_pos.header.frame_id = "wtst";
        //    obs_pos.point.x = distance * cos(direction);
        //    obs_pos.point.y = distance * sin(direction);
        obs_pos.point.x = depth + BOAT_LENGTH;
        obs_pos.point.y = depth * tan(direction);
        obs_pos.point.z = 0;
        std::cout << "obs_pos x: " << obs_pos.point.x << "  y: " << obs_pos.point.y << endl;
        obs_boat_pub.publish(obs_pos);


        VectorXd h_obs_boat(4, 1);
        h_obs_boat << obs_pos.point.x, obs_pos.point.y, 0, 1;
        VectorXd h_obs_ground = T_boat_to_ground * h_obs_boat;
        geometry_msgs::PointStamped obs_pos_g;
        obs_pos_g.header = obs_msg->header;
        obs_pos_g.header.frame_id = "map";
        obs_pos_g.point.x = h_obs_ground[0];
        obs_pos_g.point.y = h_obs_ground[1];
        obs_pos_g.point.z = h_obs_ground[2];
        std::cout << "obs_pos_ground x: " << obs_pos_g.point.x << "  y: " << obs_pos_g.point.y << endl;
        obs_ground_pub.publish(obs_pos_g);


        MeasurementPackage measurement;
        measurement.sensor_type_ = MeasurementPackage::LASER;
        measurement.timestamp_ = obs_msg->header.stamp.toSec();
        std::cout << "timestamp: " << measurement.timestamp_ << std::endl;
        measurement.raw_measurements_ = Eigen::VectorXd(2);
        measurement.raw_measurements_ << obs_pos_g.point.x, obs_pos_g.point.y;
        tracking.ProcessMeasurement(measurement);
        float x_f = tracking.kf_.x_[0];
        float y_f = tracking.kf_.x_[1];
        float v_x = tracking.kf_.x_[2];
        float v_y = tracking.kf_.x_[3];
        std::cout << "x, y after filter \n" << x_f << ", " << y_f << std::endl;
        std::cout << "vx, vy after filter \n" << v_x << ", " << v_y << std::endl;

        sensor_fusion_msg::GpsKF GPSmsg;
        GPSmsg.header = obs_msg->header;
        GPSmsg.header.frame_id = "map";
        GPSmsg.posx = tracking.kf_.x_[0];
        GPSmsg.posy = tracking.kf_.x_[1];
        GPSmsg.velx = tracking.kf_.x_[2];
        GPSmsg.vely = tracking.kf_.x_[3];

        obs_ground_filter_pub.publish(GPSmsg);

    }


}


void wtst_cb(const sailboat_message::WTST_msgConstPtr &wtst_msg) {
//    std::cout << "wtst yaw: " << wtst_msg->Yaw << std::endl;
    Eigen::Affine3d r = create_rotation_matrix(0, 0, wtst_msg->Yaw / 57.3);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(wtst_msg->PosX, wtst_msg->PosY, 0)));
    T_boat_to_ground = (t * r).matrix();


}

void img_cb(const sensor_msgs::ImageConstPtr &img_in) {
    std::cout << "===========in img callback=============" << std::endl;
    cv::Mat src_img;
    cv_bridge::toCvShare(img_in, "bgr8")->image.copyTo(src_img);

    std::cout << "======  ==========" << std::endl;
}


void obs_img_cb(const sailboat_message::Detected_obsConstPtr &obs_msg,
                const sensor_msgs::ImageConstPtr &img_msg) {
    obs_cb(obs_msg);
//    img_cb(img_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "obs_position");
    ros::NodeHandle nh;

    Eigen::MatrixXd KF_R(2, 2); //measurement covariance
    KF_R << 100, 0,
            0, 100;
    float noise_ax = 0.001;
    float noise_ay = 0.001;
    tracking.InitTracking(KF_R, noise_ax, noise_ay);

    obs_boat_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_boat_position", 2);
    obs_ground_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_ground_position", 2);
//    obs_ground_filter_pub = nh.advertise<geometry_msgs::PointStamped>("/obs_ground_filter_position", 2);
    obs_ground_filter_pub = nh.advertise<sensor_fusion_msg::GpsKF>("/obs_ground_filter_position", 2);

    ros::Subscriber wtst_sub = nh.subscribe("/wtst", 2, &wtst_cb);
    ros::Subscriber obs_sub = nh.subscribe("/detected_obs", 2, &obs_cb);

//    message_filters::Subscriber<sailboat_message::Detected_obs> obs_sub(nh, "/detected_obs", 2);
//    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/image_undistorted", 2);

//    typedef message_filters::sync_policies::ApproximateTime<sailboat_message::Detected_obs, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), obs_sub, img_sub);
//    sync.registerCallback(boost::bind(&obs_img_cb, _1, _2));


    image_transport::ImageTransport it(nh);
    pub_img = it.advertise("/camera/image_pt_reverse", 2);

    ros::spin();
    return 0;
}

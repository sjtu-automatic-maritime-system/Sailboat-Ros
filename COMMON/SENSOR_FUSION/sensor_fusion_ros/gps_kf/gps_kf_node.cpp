//
// Created by apple on 05/04/2017.
//

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "measurement_package.h"
#include "tracking.h"
#include "sensor_fusion_ros/GpsMeasurement.h"
#include "sensor_fusion_ros/GpsKF.h"
#include "sailboat_message/WTST_msg.h"
#include "ros/ros.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Create a Tracking instance

class GPS_KF {
public:
    GPS_KF() {
        GPSkf_pub = nh.advertise<sensor_fusion_ros::GpsKF>("gps_kf", 10);
        sub = nh.subscribe("WTST_tmp", 100, &GPS_KF::GPScallback, this);
//        sub = nh.subscribe("gps_measurement", 100, &GPS_KF::callback, this);

    }

    ~GPS_KF() {

    }

    void callback(const sensor_fusion_ros::GpsMeasurement::ConstPtr &msg);

    void GPScallback(const sailboat_message::WTST_msg::ConstPtr &msg);

private:
    Tracking tracking;
    ros::NodeHandle nh;
    ros::Publisher GPSkf_pub;
    ros::Subscriber sub;

};

void GPS_KF::callback(const sensor_fusion_ros::GpsMeasurement::ConstPtr &msg) {
    ROS_INFO("I head posx: %f, posy: %f", msg->posx, msg->posy);
    MeasurementPackage meas_pkg;
    if (msg->sensor_type.compare("L") == 0) {
        meas_pkg.sensor_type_ = MeasurementPackage::LASER;
        meas_pkg.timestamp_ = msg->timestamp;
        ROS_INFO("timestamp: %f", meas_pkg.timestamp_);

        meas_pkg.raw_measurements_ = VectorXd(2);
        meas_pkg.raw_measurements_ << msg->posx, msg->posy;

        //call the ProcessingMeasurement() function for each measurement
        tracking.ProcessMeasurement(meas_pkg);
        ROS_INFO("update posx: %f, posy: %f", tracking.kf_.x_[0], tracking.kf_.x_[1]);
        ROS_INFO("update velx: %f, vely: %f", tracking.kf_.x_[2], tracking.kf_.x_[3]);
        std::cout << typeid(meas_pkg.timestamp_).name() << std::endl;
        sensor_fusion_ros::GpsKF GPSmsg;
        GPSmsg.posx = tracking.kf_.x_[0];
        GPSmsg.posy = tracking.kf_.x_[1];
        GPSmsg.velx = tracking.kf_.x_[2];
        GPSmsg.vely = tracking.kf_.x_[3];

        GPSkf_pub.publish(GPSmsg);

    }
}

void GPS_KF::GPScallback(const sailboat_message::WTST_msg::ConstPtr &msg) {
    ROS_INFO("I head posx: %f, posy: %f", msg->PosX, msg->PosY);
    MeasurementPackage meas_pkg;
    meas_pkg.sensor_type_ = MeasurementPackage::LASER;
    meas_pkg.timestamp_ = msg->timestamp;
    ROS_INFO("timestamp: %f", meas_pkg.timestamp_);
    meas_pkg.raw_measurements_ = VectorXd(2);
    meas_pkg.raw_measurements_ << msg->PosX, msg->PosY;

    //call the ProcessingMeasurement() function for each measurement
    tracking.ProcessMeasurement(meas_pkg);
    ROS_INFO("update posx: %f, posy: %f", tracking.kf_.x_[0], tracking.kf_.x_[1]);
    ROS_INFO("update velx: %f, vely: %f", tracking.kf_.x_[2], tracking.kf_.x_[3]);

    sensor_fusion_ros::GpsKF GPSmsg;
    GPSmsg.posx = tracking.kf_.x_[0];
    GPSmsg.posy = tracking.kf_.x_[1];
    GPSmsg.velx = tracking.kf_.x_[2];
    GPSmsg.vely = tracking.kf_.x_[3];

    GPSkf_pub.publish(GPSmsg);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_kf_node");

    GPS_KF gps_kf;

    ros::spin();

    return 0;
}


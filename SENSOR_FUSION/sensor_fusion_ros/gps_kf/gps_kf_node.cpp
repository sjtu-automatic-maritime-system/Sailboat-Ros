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
#include "ros/ros.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Create a Tracking instance
Tracking tracking;

void callback(const sensor_fusion_ros::GpsMeasurement::ConstPtr& msg)
{
    ROS_INFO("I head posx: %f, posy: %f", msg->posx, msg->posy);
    MeasurementPackage meas_pkg;
    if (msg->sensor_type.compare("L") == 0){
        meas_pkg.sensor_type_ = MeasurementPackage::LASER;
        meas_pkg.timestamp_ = msg->timestamp;
        ROS_INFO("timestamp: %ld", meas_pkg.timestamp_);
        meas_pkg.raw_measurements_ = VectorXd(2);
        meas_pkg.raw_measurements_ << msg->posx, msg->posy;

        //call the ProcessingMeasurement() function for each measurement
        tracking.ProcessMeasurement(meas_pkg);
        ROS_INFO("update posx: %f, posy: %f", tracking.kf_.x_[0], tracking.kf_.x_[1]);
        ROS_INFO("update velx: %f, vely: %f", tracking.kf_.x_[2], tracking.kf_.x_[3]);

        }
    }


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_kf_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("gps_measurement", 100, callback);

    ros::spin();

    return 0;
}


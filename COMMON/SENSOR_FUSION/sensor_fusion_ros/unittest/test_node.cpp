//
// Created by jianyun on 17-4-5.
//

#include "ros/ros.h"
#include "sensor_fusion_ros/GpsMeasurement.h"
//#include "std_msgs/Time.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>


using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "kf_test_node");
    ros::NodeHandle nh;
    ros::Publisher kf_test_pub = nh.advertise<sensor_fusion_ros::GpsMeasurement>("gps_measurement", 100);
    ros::Rate loop_rate(10);

    /*******************************************************************************
     *  Set Measurements															 *
     *******************************************************************************/
    // hardcoded input file with laser and radar measurements
    string in_file_name_ = "/home/jianyun/catkin_ws/src/Sailboat-Ros/SENSOR_FUSION/sensor_fusion_ros/unittest/obj_pose-laser-radar-synthetic-input.txt";
    ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

    if (!in_file.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string line;

    while(ros::ok() && getline(in_file, line)){
        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;	//reads first element from the current line
        double_t timestamp;
        sensor_fusion_ros::GpsMeasurement gps_meas;
        if(sensor_type.compare("L") == 0){	//laser measurement
            //read measurements
            gps_meas.sensor_type = "L";
            float x;
            float y;
            iss >> x;
            iss >> y;
            gps_meas.posx = x;
            gps_meas.posy = y;
            iss >> timestamp;
            gps_meas.timestamp= timestamp / 1000000.0;

            ROS_INFO("timestamp %f", gps_meas.timestamp);
            kf_test_pub.publish(gps_meas);
            ros::spinOnce();
            loop_rate.sleep();

        }else if(sensor_type.compare("R") == 0){
            //Skip Radar measurements
            continue;
        }

    }

    if(in_file.is_open()){
        in_file.close();
    }
    return 0;
}


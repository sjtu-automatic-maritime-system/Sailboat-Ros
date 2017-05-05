//
// Created by hywel on 17-5-5.
//
//
#ifndef SAILBOAT_CSENSORPROCESS_H
#define SAILBOAT_CSENSORPROCESS_H

#endif //SAILBOAT_CSENSORPROCESS_H

#include "ros/ros.h"
#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "sailboat_message/Sensor_msg.h"

class CSensorProcess{
public:
    //ros相关
    ros::NodeHandle node;
    ros::Subscriber ahrs_sub;
    ros::Subscriber wtst_sub;
    ros::Publisher sensor_pub;

    CSensorProcess();
    ~CSensorProcess();

    void Init();

    //数据处理
    void ProcessMsg();

    //返回处理后的数据
    double* GetSensorMsg();

    void ahrsCallback(const sailboat_message::Ahrs_msg::ConstPtr& msg);
    void wtstCallback(const sailboat_message::WTST_msg::ConstPtr& msg);

private:
    double *AhrsMsg;
    double *WtstMsg;
    double *SensorMsg;


};

//sensorMsg = new double[11];
//float64 timestamp
//float64 ux
//float64 vy
//float64 wx
//float64 wz
//float64 Posx
//float64 Posy
//float64 Roll
//float64 Yaw
//float64 AWA
//float64 AWS

//ahrsMsg = new double[10];
//float64 timestamp
//float64 roll
//float64 pitch
//float64 yaw
//float64 gx
//float64 gy
//float64 gz
//float64 wx
//float64 wy
//float64 wz

//wtstMsg = new double[11];
//float64 timestamp
//int16 GPSIndicator
//float64 Latitude
//float64 Longitude
//float64 PosX
//float64 PosY
//float64 Roll
//float64 Pitch
//float64 Yaw
//float64 WindAngle
//float64 WindSpeed
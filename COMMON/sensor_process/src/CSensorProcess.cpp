//
// Created by hywel on 17-5-5.
//

#include "sensor_process/CSensorProcess.h"




CSensorProcess::CSensorProcess() {

    AhrsMsg = new double[10];
    WtstMsg = new double[11];
    SensorMsg = new double[11];

    Init();
}

CSensorProcess::~CSensorProcess() {
    delete [] AhrsMsg;
    delete [] WtstMsg;
    delete [] SensorMsg;
}

void CSensorProcess::Init() {
    sensor_pub = node.advertise<sailboat_message::Sensor_msg>("sensor", 2);
    ahrs_sub = node.subscribe("ahrs", 2, &CSensorProcess::ahrsCallback,this);
    wtst_sub = node.subscribe("wtst", 2, &CSensorProcess::wtstCallback,this);
}

void CSensorProcess::ProcessMsg() {
    SensorMsg[1] = 0; //todo!
    SensorMsg[2] = 0; //todo!
    SensorMsg[3] = AhrsMsg[7];
    SensorMsg[4] = AhrsMsg[9];
    SensorMsg[5] = WtstMsg[4];
    SensorMsg[6] = WtstMsg[5];
    SensorMsg[7] = AhrsMsg[1];
    SensorMsg[8] = AhrsMsg[3];
    SensorMsg[9] = WtstMsg[9];
    SensorMsg[10] = WtstMsg[10];
}

double* CSensorProcess::GetSensorMsg() {
    ProcessMsg();
    return SensorMsg;
}


void CSensorProcess::ahrsCallback(const sailboat_message::Ahrs_msg::ConstPtr &msg) {
    ROS_INFO("ahrs_msg sub: [%f] [%f] [%f]", msg->roll,msg->pitch,msg->yaw);
    AhrsMsg[0] = msg->timestamp;
    AhrsMsg[1] = msg->roll;
    AhrsMsg[2] = msg->pitch;
    AhrsMsg[3] = msg->yaw;
    AhrsMsg[4] = msg->gx;
    AhrsMsg[5] = msg->gy;
    AhrsMsg[6] = msg->gz;
    AhrsMsg[7] = msg->wx;
    AhrsMsg[8] = msg->wy;
    AhrsMsg[9] = msg->wz;
}

void CSensorProcess::wtstCallback(const sailboat_message::WTST_msg::ConstPtr& msg)
{
    ROS_INFO("wtst_msg sub: [%f] [%f]", msg->PosX,msg->PosY);
    WtstMsg[0] = msg->timestamp;
    WtstMsg[1] = msg->GPSIndicator;
    WtstMsg[2] = msg->Latitude;
    WtstMsg[3] = msg->Longitude;
    WtstMsg[4] = msg->PosX;
    WtstMsg[5] = msg->PosY;
    WtstMsg[6] = msg->Roll;
    WtstMsg[7] = msg->Pitch;
    WtstMsg[8] = msg->Yaw;
    WtstMsg[9] = msg->WindAngle;
    WtstMsg[10] = msg->WindSpeed;
}

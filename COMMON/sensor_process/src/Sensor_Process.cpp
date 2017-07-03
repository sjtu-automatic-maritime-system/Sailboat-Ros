//
// Created by hywel on 17-5-4.
//

// sensor_msg 输出的角度为弧度制




#include "sensor_process/CSensorProcess.h"


//float64 timestamp
//float64 ux
//float64 vy
//float64 gx
//float64 gz
//float64 Posx
//float64 Posy
//float64 Roll
//float64 Yaw
//float64 AWA
//float64 AWS



int main(int argc, char **argv)
{

    ros::init(argc, argv, "sensor_process");

    CSensorProcess senPro;

    ros::Rate loop_rate(10);
    double *sensorMsg;

    while (ros::ok())
    {

        sensorMsg = senPro.GetSensorMsg();
        sailboat_message::Sensor_msg msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "sensor";

        msg.timestamp = ros::Time::now().toSec();
        msg.ux = sensorMsg[1];
        msg.vy = sensorMsg[2];
        msg.gx = sensorMsg[3];
        msg.gz = sensorMsg[4];
        msg.Posx = sensorMsg[5];
        msg.Posy = sensorMsg[6];
        msg.Roll = sensorMsg[7];
        msg.Yaw = sensorMsg[8];
        msg.AWA = sensorMsg[9];
        msg.AWS = sensorMsg[10];

        senPro.sensor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}
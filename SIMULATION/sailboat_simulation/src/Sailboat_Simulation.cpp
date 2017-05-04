//
// Created by hywel on 17-5-2.
//

#include "CSimulationVer1.h"

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "SimulationVer1");

    CSimulationVer1 SME;

    ros::Rate loop_rate(50);

    double* sailboat_msg;
    sailboat_msg = new double[10];

    //设置帆船初始状态
    SME.SettingAttitudeInit(0,0,0,0,0,0,0,pi);

    while (ros::ok())
    {
        SME.Sailboat_Calc(0.02);
        sailboat_msg = SME.Sailboat_Out();

        sailboat_message::Sensor_msg msg;

        msg.ux = sailboat_msg[0];
        msg.vy = sailboat_msg[1];
        msg.wx = sailboat_msg[2];
        msg.wz = sailboat_msg[3];
        msg.Posx = sailboat_msg[4];
        msg.Posy = sailboat_msg[5];
        msg.Roll = sailboat_msg[6];
        msg.Yaw = sailboat_msg[7];
        msg.AWA = sailboat_msg[8];
        msg.AWS = sailboat_msg[9];

        ROS_INFO("I talk Sailboat_Simulation: [%f] [%f]", msg.Posx, msg.Posy);
        SME.sensor_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;

}

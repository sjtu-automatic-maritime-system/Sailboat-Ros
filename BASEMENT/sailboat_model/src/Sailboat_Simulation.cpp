//
// Created by hywel on 17-5-2.
//

#include "CSailboatMotionEquation.h"

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "SailboatSimulation");

    CSailboatMotionEquation SME;

    ros::Rate loop_rate(10);

    double* sailboat_msg;
    sailboat_msg = new double[8];

    while (ros::ok())
    {
        SME.Sailboat_Calc();
        sailboat_msg = SME.Sailboat_Out();

        sailboat_message::Sailboat_Simulation_msg msg;

        msg.ux = sailboat_msg[0];
        msg.vy = sailboat_msg[1];
        msg.wx = sailboat_msg[2];
        msg.wz = sailboat_msg[3];
        msg.posx = sailboat_msg[4];
        msg.posy = sailboat_msg[5];
        msg.yaw = sailboat_msg[6];
        msg.roll = sailboat_msg[7];

        ROS_INFO("I talk Sailboat_Simulation: [%f] [%f]", msg.posx, msg.posy);
        SME.sailboat_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;

}

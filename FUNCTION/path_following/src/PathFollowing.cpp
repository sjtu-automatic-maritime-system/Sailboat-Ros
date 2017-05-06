//
// Created by hywel on 17-5-6.
//

#include "path_following_lib/CPathFollowingVer1.h"

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "path_following_ver1");

    CPathFollowingVer1 pathfollowing;

    double *x;
    double *y;
    x = new double[8];
    y = new double[8];
    double xx[8] = {-10,-20,-30,-40,-40,-30,-20,-10};
    double yy[8] = {0,-5,-15,-10,10,15,5,0};
    for (int i = 0; i < 8; ++i) {
        x[i] = xx[i];
        y[i] = yy[i];
        //cout<<x[i]<<endl;
    }

    pathfollowing.Init(x,y,8);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        double target;
        target = pathfollowing.CalcFor();

        sailboat_message::Target_msg msg;
        msg.timestamp = ros::Time::now().toSec();
        msg.TargetAngle = target;
        //ROS_INFO("I talk Rudder_Angle: [%f]", msg.rudder);
        ROS_INFO("I talk TargetAngle: [%f]", msg.TargetAngle);
        pathfollowing.targetAngle_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

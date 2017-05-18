//
// Created by hywel on 17-5-6.
//

#include "path_following_lib/CPathFollowingVer1.h"
#include <dynamic_reconfigure/server.h>
#include "path_following/path_following_point_Config.h"

static double minJudgeDistence = 5;

static int pointx1 = -10;
static int pointy1 = 0;
static int pointx2 = -20;
static int pointy2 = -5;
static int pointx3 = -30;
static int pointy3 = -15;
static int pointx4 = -40;
static int pointy4 = -10;
static int pointx5 = -40;
static int pointy5 = 10;
static int pointx6 = -30;
static int pointy6 = 15;
static int pointx7 = -20;
static int pointy7 = 5;
static int pointx8 = -10;
static int pointy8 = 0;

static int isChange = 0;
static bool isChange_old = true;

void callback(path_following::path_following_point_Config &config, uint32_t level) {
    //ROS_INFO("Reconfigure Request: %f %f %d",
    //       config.Sail_Angle,config.Rudder_Angle,
    //       config.size);
    minJudgeDistence = config.minJudgeDistence;

    pointx1 = config.point1_x;
    pointy1 = config.point1_y;
    pointx2 = config.point2_x;
    pointy2 = config.point2_y;
    pointx3 = config.point3_x;
    pointy3 = config.point3_y;
    pointx4 = config.point4_x;
    pointy4 = config.point4_y;
    pointx5 = config.point5_x;
    pointy5 = config.point5_y;
    pointx6 = config.point6_x;
    pointy6 = config.point6_y;
    pointx7 = config.point7_x;
    pointy7 = config.point7_y;
    pointx8 = config.point8_x;
    pointy8 = config.point8_y;

    if(config.changePoint != isChange_old)
        isChange = 1;
        isChange_old = config.changePoint;

}


int main(int argc, char **argv)
{

    //Initiate ROS
    ros::init(argc, argv, "path_following_ver1");
    CPathFollowingVer1 pathfollowing;

    double *x;
    double *y;
    x = new double[8];
    y = new double[8];

    double xx[8];
    double yy[8];
    xx[0] = pointx1*1.0;
    yy[0] = pointy1*1.0;
    xx[1] = pointx2*1.0;
    yy[1] = pointy2*1.0;
    xx[2] = pointx3*1.0;
    yy[2] = pointy3*1.0;
    xx[3] = pointx4*1.0;
    yy[3] = pointy4*1.0;
    xx[4] = pointx5*1.0;
    yy[4] = pointy5*1.0;
    xx[5] = pointx6*1.0;
    yy[5] = pointy6*1.0;
    xx[6] = pointx7*1.0;
    yy[6] = pointy7*1.0;
    xx[7] = pointx8*1.0;
    yy[7] = pointy8*1.0;
    for (int i = 0; i < 8; ++i) {
        x[i] = xx[i];
        y[i] = yy[i];
        //cout<<x[i]<<endl;
    }

    pathfollowing.Init(x,y,8);


    dynamic_reconfigure::Server<path_following::path_following_point_Config> dserver;
    dynamic_reconfigure::Server<path_following::path_following_point_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    dserver.setCallback(f);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pathfollowing.SetMinJudgeDistence(minJudgeDistence);

        if(isChange == 1)
        {
            xx[0] = pointx1*1.0;
            yy[0] = pointy1*1.0;
            xx[1] = pointx2*1.0;
            yy[1] = pointy2*1.0;
            xx[2] = pointx3*1.0;
            yy[2] = pointy3*1.0;
            xx[3] = pointx4*1.0;
            yy[3] = pointy4*1.0;
            xx[4] = pointx5*1.0;
            yy[4] = pointy5*1.0;
            xx[5] = pointx6*1.0;
            yy[5] = pointy6*1.0;
            xx[6] = pointx7*1.0;
            yy[6] = pointy7*1.0;
            xx[7] = pointx8*1.0;
            yy[7] = pointy8*1.0;

            for (int i = 0; i < 8; ++i) {
                x[i] = xx[i];
                y[i] = yy[i];
                //cout<<x[i]<<endl;
            }

            pathfollowing.Init(x,y,8);
            isChange = 0;
        }


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

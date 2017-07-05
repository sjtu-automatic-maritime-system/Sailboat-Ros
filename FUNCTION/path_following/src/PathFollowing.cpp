//
// Created by hywel on 17-5-6.
//

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"//改成处理过后的Sensor_msg
#include "sailboat_message/Target_msg.h"

#include "path_following_lib/CPathFollowingVer1.h"
#include <dynamic_reconfigure/server.h>
#include "path_following/path_following_point_Config.h"

static CPathFollowingVer1 pathfollowingver1;

static double xx[8];
static double yy[8];

static int isChange = 0;
static bool isChange_old = true;


void SensorCallback(const sailboat_message::Sensor_msg::ConstPtr &msg) {
    ROS_INFO("I get AWA: [%f]", msg->AWA);
    pathfollowingver1.ux = msg->ux;
    pathfollowingver1.vy = msg->vy;
    pathfollowingver1.wx = msg->gx;
    pathfollowingver1.wz = msg->gz;
    pathfollowingver1.posX = msg->Posx;
    pathfollowingver1.posY = msg->Posy;
    pathfollowingver1.Roll = msg->Roll;
    pathfollowingver1.Yaw= msg->Yaw;
}


void PointCallback(path_following::path_following_point_Config &config, uint32_t level) {

    pathfollowingver1.minJudgeDistence = config.minJudgeDistence;

    //cpathfollowing中，坐标点个数任意，但config中坐标点个数有限
    xx[0] = config.point0_x;
    yy[0] = config.point0_y;
    xx[1] = config.point1_x;
    yy[1] = config.point1_y;
    xx[2] = config.point2_x;
    yy[2] = config.point2_y;
    xx[3] = config.point3_x;
    yy[3] = config.point3_y;
    xx[4] = config.point4_x;
    yy[4] = config.point4_y;
    xx[5] = config.point5_x;
    yy[5] = config.point5_y;
    xx[6] = config.point6_x;
    yy[6] = config.point6_y;
    xx[7] = config.point7_x;
    yy[7] = config.point7_y;

    ROS_INFO("Reconfigure Request: %f %f", xx[0],yy[0]);

    if(config.changePoint != isChange_old){
        isChange = 1;
        isChange_old = config.changePoint;
    }

    if(isChange == 1)
    {
        pathfollowingver1.Init(xx,yy,8);
        isChange = 0;

    }
}

void Target_pub(sailboat_message::Target_msg& msg){
    msg.timestamp = ros::Time::now().toSec();
    msg.TargetAngle = pathfollowingver1.targetAngle;

    ROS_INFO("I talk TargetAngle: [%f]", msg.TargetAngle);
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "path_following_ver1");

    ros::NodeHandle pf_node;
    ros::Subscriber sensor_sub;
    ros::Publisher targetAngle_pub;

    targetAngle_pub = pf_node.advertise<sailboat_message::Target_msg>("targetangle", 5);
    sensor_sub = pf_node.subscribe("sensor", 2, SensorCallback);

    dynamic_reconfigure::Server<path_following::path_following_point_Config> dserver;
    dynamic_reconfigure::Server<path_following::path_following_point_Config>::CallbackType f;
    f = boost::bind(&PointCallback, _1, _2);
    dserver.setCallback(f);

    pathfollowingver1.Init(xx,yy,8);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pathfollowingver1.CalcFor();

        sailboat_message::Target_msg msg;
        Target_pub(msg);
        targetAngle_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

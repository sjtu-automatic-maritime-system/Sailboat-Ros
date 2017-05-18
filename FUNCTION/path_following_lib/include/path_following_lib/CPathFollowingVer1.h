//
// Created by hywel on 17-5-6.
//
//输入的坐标点 最好为不小于10m间距的点，相邻三个点的夹角大于90度
//原则，不拐小于90度的弯
//暂时不考虑计算效率

#ifndef SAILBOAT_CPATHFOLLOWVER2_H
#define SAILBOAT_CPATHFOLLOWVER2_H

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"//改成处理过后的Sensor_msg
#include "sailboat_message/Target_msg.h"
#include <iostream>
#include <cmath>

#define pi 3.1415926

using namespace std;

class CPathFollowingVer1{
public:

    ros::NodeHandle pf_node;
    ros::Subscriber sensor_sub;
    ros::Publisher targetAngle_pub;

    CPathFollowingVer1();
    ~CPathFollowingVer1();

    //设置路径
    void Init(double* x, double *y,int num);

    void JudgeReach();
    void FindTarget();

    void SetMinJudgeDistence(double d);

    void CalcTurningRadius();
    //没有用到
    void CalcHeadingDeviation();

    void CalcTargetAngle();

    double CalcFor();

    void UpdateSenserDate(double uxx, double vyy, double wxx, double wzz, double posx, double posy, double roll, double yaw);

    void SensorCallback(const sailboat_message::Sensor_msg::ConstPtr &msg);

private:
    //判断参数
    bool isPathFollowing;
    double sailboatTurnR;
    double sailboatL;
    int toPointId;
    int oldToPointId;

    double judgeDistence;
    double minJudgeDistence;

    //计算
    double targeDistance;

    double turningAngle;
    double turningRadius;

    double headingDeviation;
    //输出
    double targetAngle;
    //输入
    //X -10,-20,-30,-40,-40,-30,-20,-10
    //Y 0,-5,-15,-10,10,15,5,0
    double *pathX;
    double *pathY;

    int count;
    //传感器数据
    double ux;
    double vy;
    double wx;
    double wz;
    double posX;
    double posY;
    double Roll;
    double Yaw;
};

































#endif //SAILBOAT_CPATHFOLLOWVER2_H

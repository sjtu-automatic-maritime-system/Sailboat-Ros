//
// Created by hywel on 17-5-6.
//
//输入的坐标点 最好为不小于10m间距的点，相邻三个点的夹角大于90度
//原则，不拐小于90度的弯
//暂时不考虑计算效率

#ifndef SAILBOAT_CPATHFOLLOWVER2_H
#define SAILBOAT_CPATHFOLLOWVER2_H


#include <iostream>
#include <cmath>

#define pi 3.1415926

using namespace std;

class CPathFollowingVer1{
public:

    CPathFollowingVer1();
    ~CPathFollowingVer1();

    //设置路径
    void Init(double* x, double *y,int num);

    void JudgeReach();
    void FindTarget();


    void CalcTurningRadius();
    //没有用到
    void CalcHeadingDeviation();

    void CalcTargetAngle();

    void CalcFor();

    double ux;
    double vy;
    double wx;
    double wz;
    double posX;
    double posY;
    double Roll;
    double Yaw;

    double targetAngle;

    double minJudgeDistence;

private:
    //判断参数
    bool isPathFollowing;
    double sailboatTurnR;
    double sailboatL;
    int toPointId;
    int oldToPointId;

    double judgeDistence;


    //计算
    double targeDistance;

    double turningAngle;
    double turningRadius;

    double headingDeviation;
    //输出

    //输入
    //X -10,-20,-30,-40,-40,-30,-20,-10
    //Y 0,-5,-15,-10,10,15,5,0
    double *pathX;
    double *pathY;

    int count;
    //传感器数据

};

































#endif //SAILBOAT_CPATHFOLLOWVER2_H

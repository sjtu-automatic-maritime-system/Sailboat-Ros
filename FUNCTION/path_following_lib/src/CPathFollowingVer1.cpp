//
// Created by hywel on 17-5-6.
//

#include "path_following_lib/CPathFollowingVer1.h"

CPathFollowingVer1::CPathFollowingVer1() {
    sailboatTurnR = 8;
    sailboatL = 1.5;
    toPointId = 0;
    oldToPointId = -1;

    turningAngle = 0;
    turningRadius = 0;
    headingDeviation = 0;

    minJudgeDistence = 5;

    targetAngle_pub = pf_node.advertise<sailboat_message::Target_msg>("targetangle", 5);
    sensor_sub = pf_node.subscribe("sensor", 2, &CPathFollowingVer1::SensorCallback,this);
}

CPathFollowingVer1::~CPathFollowingVer1() {
    delete [] pathX;
    delete [] pathY;
}

void CPathFollowingVer1::Init(double *x, double *y,int num) {

    pathX = new double[num];
    pathY = new double[num];

    for (int i = 0; i < num; ++i) {
        pathX[i] = x[i];
        pathY[i] = y[i];
        cout<<pathX[i]<<endl;
    }
    count = num;
    ROS_INFO("count = [%i]",count);


}

void CPathFollowingVer1::SetMinJudgeDistence(double d)
{
    minJudgeDistence = d;
}

void CPathFollowingVer1::UpdateSenserDate(double uxx, double vyy, double wxx, double wzz, double posx, double posy,
                                          double roll, double yaw) {
    ux = uxx;
    vy = vyy;
    wx = wxx;
    wz = wzz;
    posX = posx;
    posY = posy;
    Roll = roll;
    Yaw = yaw;
}

void CPathFollowingVer1::JudgeReach() {
    double x0 = posX;
    double y0 = posY;
    double x2 = pathX[toPointId];
    double y2 = pathY[toPointId];
    targeDistance = sqrt(pow(x2-x0,2)+pow(y2-y0,2));

    judgeDistence = turningRadius;
    if(judgeDistence < minJudgeDistence)
        judgeDistence = minJudgeDistence;

    if(judgeDistence > targeDistance)
    {
        oldToPointId = toPointId;
        toPointId = toPointId + 1;
    }
    ROS_INFO("[%i] [%f] [%f]",toPointId,pathX[toPointId],pathY[toPointId]);

}

void CPathFollowingVer1::FindTarget() {
    if(toPointId == count-1 && pathX[0] == pathX[count-1] &&pathY[0] == pathY[count-1])
        toPointId = 0;
    if(toPointId > count-1){
        toPointId = 0;
        ROS_INFO("finish");
        isPathFollowing = false;
    }
}

void CPathFollowingVer1::CalcTurningRadius() {
    double x0 = posX;
    double y0 = posY;
    double x2 = pathX[toPointId];
    double y2 = pathY[toPointId];
    double x3 = pathX[toPointId + 1];
    double y3 = pathY[toPointId + 1];

    turningAngle = acos(((x2 - x0) * (x3 - x2) + (y2 - y0) * (y3 - y2)) /
                   sqrt((pow(x2 - x0, 2) + pow(y2 - y0, 2)) * (pow(x3 - x2, 2) + pow(y3 - y2, 2))));
    turningRadius = sailboatTurnR/tan(0.5*(pi-turningAngle))+sailboatL;
}

void CPathFollowingVer1::CalcHeadingDeviation() {
    double x0 = posX;
    double y0 = posY;
    double x1 = pathX[oldToPointId];
    double y1 = pathY[oldToPointId];
    double x2 = pathX[toPointId];
    double y2 = pathY[toPointId];

    headingDeviation =- ((y2-y1)*x0+(x1-x2)*y0-x1*y2+x2*y1)/sqrt(pow(y2-y1,2)+pow(x2-x1,2));
}

void CPathFollowingVer1::CalcTargetAngle() {

    double x0 = posX;
    double y0 = posY;
    double x2 = pathX[toPointId];
    double y2 = pathY[toPointId];
    targetAngle = atan2((y2-y0),(x2-x0));
    //if(fabs(targetAngle)>3)
    //{
        //targetAngle =3;
    //}
}

double CPathFollowingVer1::CalcFor(){
    JudgeReach();
    FindTarget();

    CalcTurningRadius();
    CalcHeadingDeviation();

    CalcTargetAngle();
    return targetAngle;
}


void CPathFollowingVer1::SensorCallback(const sailboat_message::Sensor_msg::ConstPtr &msg) {
    //ROS_INFO("I get AWA: [%f]", msg->AWA);
    ux = msg->ux;
    vy = msg->vy;
    wx = msg->wx;
    wz = msg->wz;
    posX = msg->Posx;
    posY = msg->Posy;
    Roll = msg->Roll;
    Yaw= msg->Yaw;
}

//
// Created by hywel on 17-4-23.
//

#include "autopilot/CAutopilotVer1.h"


CAutopilotVer1::CAutopilotVer1() {

    Kp = 0.5;
    Ki = 0;
    Kd = 0;
    T = 0.1;
    OutMax = 0.52;
    OutMin = -0.52;
    yawFdb = 0;
    yawRef = 0;

    PCCtrl = 0;
    rudder = 0;
    sail = 0;

    Init();

}

CAutopilotVer1::CAutopilotVer1(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = 0.1;
    OutMax = 0.52;
    OutMin = -0.52;
    yawFdb = 0;
    yawRef = 0;

    PCCtrl = 0;
    rudder = 0;
    sail = 0;

    Init();
}

CAutopilotVer1::CAutopilotVer1(double kp, double ki, double kd, double t, double outMax, double outMin) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = t;
    OutMax = outMax;
    OutMin = outMin;
    yawFdb = 0;
    yawRef = 0;

    PCCtrl = 0;
    rudder = 0;
    sail = 0;

    Init();
}

CAutopilotVer1::~CAutopilotVer1() {
    delete pidp;

}

double CAutopilotVer1::Get_Rudder() {
    return rudder;
}

double CAutopilotVer1::Get_Sail() {
    return sail;
}
int CAutopilotVer1::Get_PCCtrl(){
    return PCCtrl;
}

void CAutopilotVer1::Init() {
    pidp=new CPID(Kp,Ki, Kd, T, OutMax,OutMin);
}

void CAutopilotVer1::set_Pid(){
    pidp->Set_Kp(Kp);
    pidp->Set_Ki(Ki);
    pidp->Set_Kd(Kd);
}


void CAutopilotVer1::AP_Calc() {
    pidp->Set_Ref(yawRef);
    pidp->Set_Fdb(yawFdb);
    //ROS_INFO("YAWRef and YAWFdb: [%f] [%f]",yawRef,yawFdb);
    //rudder = pidp->PID_Calc();

    rudder = (yawRef-yawFdb)*Kp;
    sail = sailCtrl.GetBestSailAngle2(AWA);
    if (sail> pi/2)
        sail = pi/2;
    else if (sail < -pi/2)
        sail = -pi/2;

    //ROS_INFO("sail: [%f]",sail);

}

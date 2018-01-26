//
// Created by hywel on 17-4-23.
//

#ifndef SAILBOAT_CAUTOPILOTVER1_H
#define SAILBOAT_CAUTOPILOTVER1_H

#include "math_tool_lib/CPID.h"
#include "autopilot/CSailCtrl.h"
#include <sstream>

#define pi 3.1415926

class CAutopilotVer1 {
public:


    CAutopilotVer1();
    CAutopilotVer1(double kp, double ki, double kd);
    CAutopilotVer1(double kp, double ki, double kd, double t, double outMax, double outMin);
    ~CAutopilotVer1();

    //初始化
    void Init();

    //得到舵角或帆角
    double Get_Rudder();
    double Get_Sail();
    int Get_PCCtrl();
    //pid算法得到舵角
    void set_Pid();
    void AP_Calc();


//private:

    CSailCtrl sailCtrl;

    CPID* pidp;


    int PCCtrl;
    double Kp;
    double Ki;
    double Kd;
    double T;
    double OutMax;
    double OutMin;

    double yawRef;
    double yawFdb;

    double AWA;
    double AWS;

    double rudder;
    double sail;

};


#endif //SAILBOAT_CAUTOPILOTVER1_H

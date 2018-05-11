//
// Created by hywel on 17-4-23.
//

#include "math_tool_lib/CPID.h"


CPID::CPID() {
    Ref = 0;
    Fdb = 0;
    Kp = 2;
    Ki = 0;
    Kd = 0;
    T = 0.01;
    a0 = 0;
    a1 = 0;
    a2 = 0;
    Error = 0;
    Error_1 = 0;
    Error_2 = 0;
    Out = 0;
    Out_1 = 0;
    OutMax = 10;
    OutMin = -10;
}

CPID::CPID(double kp, double ki, double kd, double t, double outMax, double outMin) {
    Ref = 0;
    Fdb = 0;
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = t;
    a0 = 0;
    a1 = 0;
    a2 = 0;
    Error = 0;
    Error_1 = 0;
    Error_2 = 0;
    Out = 0;
    Out_1 = 0;
    OutMax = outMax;
    OutMin = outMin;
}

CPID::~CPID() {

}

void CPID::Set_Kp(double kp) {
    Kp = kp;
}

void CPID::Set_Ki(double ki) {
    Ki = ki;
}

void CPID::Set_Kd(double kd) {
    Kd = kd;
}

double CPID::Get_Kpid() {
    return Kp, Ki, Kd;
}

void CPID::Set_Fdb(double fdb) {
    Fdb = fdb;
}

void CPID::Set_Ref(double ref) {
    Ref = ref;
}

double CPID::Get_Out() {
    return Out;
}

double CPID::PID_Calc() {

    Error = Ref - Fdb;
    while (fabs(Error)>PI)
    {
        if (Error >= PI)
            Error = Error - 2*PI;
        else if (Error <= -PI)
            Error = Error + 2*PI;
    }
    //计算中间量 a0a1a2
    a0 = Kp+Ki*T+Kd/T;
    a1 = Kp+2*Kd/T;
    a2 = Kd/T;

    //计算PID控制其的输出
    Out = Out_1+a0*Error-a1*Error_1+a2*Error_2;

    //输出限幅
    if(Out > OutMax)
        Out = OutMax;
    if(Out < OutMin)
        Out = OutMin;

    //为下一步计算做准备
    Out_1 = Out;
    Error_2 = Error_1;
    Error_1 = Error;

    return Out;
}






//
// Created by hywel on 17-4-23.
//

#ifndef SAILBOAT_CPID_H
#define SAILBOAT_CPID_H

#include <cmath>

#ifndef PI
#define PI 3.1415926
#endif

//增量式PID
//http://www.cnblogs.com/cjq0301/p/5184808.html
//u(k) = u(k-1)+(kp+ki*T+kd/T)e(k)-(kp+2kd/T)e(k-1)+kd/T*e(k-2)
class CPID {

public:
    CPID();
    CPID(double kp, double ki, double kd, double t, double outMax, double outMin);
    ~CPID();

    void Set_Kp(double kp);

    void Set_Ki(double ki);

    void Set_Kd(double kd);

    double Get_Kpid();

    void Set_Ref(double ref);

    void Set_Fdb(double fdb);

    double Get_Out();

    double PID_Calc();

    //void Pid_demo();
private:
    double Ref;    //输入：系统待调节量的给定值
    double Fdb;    //输入：系统待调节量的反馈值

    double Kp;     //参数
    double Ki;     //参数
    double Kd;     //参数

    double T;      //参数：离散化系统的采样周期

    double a0;     //变量
    double a1;     //变量
    double a2;     //变量

    double Error;  // 变量：当前的偏差e(k)
    double Error_1;// 历史：前一步的偏差e(k-1)
    double Error_2;// 历史：前前一步的偏差e(k-2)

    double Out;    //输出：PID控制器的输出u(k)
    double Out_1;  //历史：u(k-1)
    double OutMax; //参数
    double OutMin; //参数

};


#endif //SAILBOAT_CPID_H

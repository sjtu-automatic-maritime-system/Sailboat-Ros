//
// Created by hywel on 17-4-25.
// reference Jianyun Xu's Simulink
//
// 一个找了一天的bug：定义数组指针后必须new分配内存。
#ifndef SAILBOAT_CSIMULATIONVER1_H
#define SAILBOAT_CSIMULATIONVER1_H

//math
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cfloat>

#include "CCubicSplineInterpolation.h"

//matrix
#include <Eigen/Dense>

//ros
#include "ros/ros.h"
#include "sailboat_message/Target_msg.h"
#include "sailboat_message/Sensor_msg.h"
//#include "sailboat_message/Sailboat_Simulation_msg.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include "sailboat_message/Mach_msg.h"

#define pi 3.1415926

//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace std;
//所有角度传输时，计算时再转换成弧度.
//最终输入输出为角度

struct param{
    //---parinit----
    double g;
    double rou; //kg/m^3 density of water
    double roul; //kg/m^3 density of air

    //sailboat
    double m; //kg; mass of sailboat
    double Ix;
    double Iz;
    double Ixz; //kgm^2; inertia moment of sailboat
    double x_g;
    double y_g;
    double z_g; //m; coords of gravity center
    double a11;
    double a22;
    double a24;
    double a26;
    double a44;
    double a46;
    double a66; //kg,kgm,kgm^2; added mass coff
    double GMt; //m

    //keel
    double Ak; //m^2
    double x_k;
    double y_k;
    double z_k;

    //hull
    double Ah; //m^2
    double x_h;
    double y_h;
    double z_h;
    double k1;
    double k2;

    //Rudder
    double Ar;
    double x_r;
    double y_r;
    double z_r;

    //Sail
    //Main Sail
    double Asm;
    double pm1;
    double pm2;
    double z_sm;
    //Fore Sail
    double Asf;
    double pf1;
    double pf2;
    double z_sf;
};

class CSimulationVer1{
public:
    //ros相关
    ros::NodeHandle simulation_node;
    ros::Subscriber wind_sub;
    ros::Subscriber mach_sub;
    ros::Publisher sensor_pub;
    //sailboat_message::Sailboat_Simulation_msg ssmsg;

    CSimulationVer1();
    ~CSimulationVer1();

    //setting 帆船初始姿态
    void SettingAttitudeInit(double u, double v, double p, double r, double x, double y, double phi, double psi);
    //ros节点初始化
    void Init();

    //coef系数初始化
    void KeelCoefInit();
    void HullCoefInit();
    void RudderCoefInit();
    void SailCoefInit();

    double* KeelCoef(double alpha);
    double* HullCoef(double alpha);
    double* RudderCoef(double alpha);
    double* SailCoef(double alpha);

    //各项力计算
    void RestoreForce();
    void DampingForce();
    double* Damping(double rou, double A, double x, double y, double z, int id);
    void ControlForceRudder();
    void ControlForceSail();

    //4DOF方程
    void Equation();

    //弧度角度转化
    double d2r(double d);
    double r2d(double r);
    //限幅
    double limitsPi(double tmp);

    // 计算和循环
    void Sailboat_Test(double time, double d_t);
    void Sailboat_Calc(double d_t);

    double* Sailboat_Out();

    //ros callback函数
    void WindCallback(const sailboat_message::Wind_Simulation_msg::ConstPtr& msg);
    void MachCallback(const sailboat_message::Mach_msg::ConstPtr& msg);


private:

    //参数和变量
    //double t;
    //double delta_t; //时间步长delta_t

    MatrixXd g_eta;
    MatrixXd D_nu;
    MatrixXd tau_r;
    MatrixXd tau_s;
    MatrixXd F;

    MatrixXd M_RB;
    MatrixXd C_RB;
    MatrixXd M_A;
    MatrixXd C_A;
    MatrixXd J;

    MatrixXd nu_dot;
    MatrixXd eta_dot;

    //MatrixXd v_in;
    MatrixXd nu;
    MatrixXd eta;

    //MatrixXd v_out;

    CCubicSplineInterpolation* CsipKeelYl;
    CCubicSplineInterpolation* CsipKeelYd;
    CCubicSplineInterpolation* CsipHullYl;
    CCubicSplineInterpolation* CsipHullYd;
    CCubicSplineInterpolation* CsipRudderYl;
    CCubicSplineInterpolation* CsipRudderYd;
    CCubicSplineInterpolation* CsipSailYl;
    CCubicSplineInterpolation* CsipSailYd;

    double uu;
    double vv;
    double pp;
    double rr;
    double XX;
    double YY;
    double phi;
    double psi;

    double AWA;
    double AWS;

    double rudderAngle;
    double sailAngle;
    double windDirection;
    double windVelocity;

    double delta_r;
    double delta_s;
    //damping
    //double *D_keel_tmp;
    //double *D_hull_tmp;

    param *par;

};





#endif //SAILBOAT_CSIMULATIONVER2_H


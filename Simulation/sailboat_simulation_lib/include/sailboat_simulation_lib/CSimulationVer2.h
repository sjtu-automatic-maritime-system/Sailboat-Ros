//
// Created by hywel on 17-5-4.
// reference Mingshu Du's Simulink
// to do!!! ros
//

#ifndef SAILBOAT_CSIMULATIONVER2_H
#define SAILBOAT_CSIMULATIONVER2_H

#endif //SAILBOAT_CSIMULATIONVER2_H

#include <cmath>
#include <cstring>
//matrix
#include <eigen3/Eigen/Dense>

#include "math_tool_lib/CCubicSplineInterpolation.h"


#define PI 3.1415926

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace std;

class param{
public:

    param();
    ~param();

    double g;
    double rouwater;
    double rouair;
    // hydrostatics
    double m;
    double mx;
    double my;
    double mz;
    double Ixx;
    double Iyy;
    double Izz;
    double Jxx;
    double Jyy;
    double Jzz;
    double lwl;
    double draft;
    double GMt;
    double areawethull;
    double draftwithkeel;
    // hull & keel
    double qf;
    double qh;
    double qh2;
    double qh3;
    double Xvdpsi;
    double Ydfi;
    double Ydpsi;
    double Kdfi;
    double Ndpsi;
    double Xvv;
    double Xfifi;
    double Xvvvv;
    double Yv;
    double Yfi;
    double Yvfifi;
    double Yvvfi;
    double Yvvv;
    double Kv;
    double Kfi;
    double Kvfifi;
    double Kvvfi;
    double Kvvv;
    double Nv;
    double Nfi;
    double Nvfifi;
    double Nvvfi;
    double Nvvv;
    // sail
    double areasail;
    double msail;
    double sail_xcg;
    double sail_ycg;
    double sail_zcg;
    double sail_xgce;
    double sail_ygce;
    double sail_zgce;
    // rudder
    double arearudder;
    double rudder_xgce;
    double rudder_ygce;
    double rudder_zgce;
    double Cxrudder;
    double Cyrudder;
    double Ckrudder;
    double Cnrudder;
    //keel
    double areakeel;
    double keel_xgce;
    double keel_ygce;
    double keel_zgce;
};

class CSimulationVer2{
public:
    CSimulationVer2();
    ~CSimulationVer2();


    void CalcAWSAWA();
    //帆船4种受力计算
    double X004();
    double* FM_add();
    double* FM_rudder();
    double* FM_sail();
    //coef系数计算
    double coefLTH(double fi, double beta , double Fn);
    double coefRTH(double fi, double beta , double Fn);
    double resist0H(double Vyacht);

    void RudderCoefInit();
    void SailCoefInit();
    double* RudderCoef(double alpha);
    double* SailCoef(double alpha);

    //4DOF 方程计算
    void FourD0F();
    //限幅
    double limitsPi(double tmp);
    //计算地球坐标系的x,y方向速度
    double* transformation();


    //以下需要调用的函数
    //最终计算
    void Sailboat_Calc(double d_t);
    void SetOrigin(double u0, double v0, double phi0, double psi0, double e0, double n0);


private:

    double dphi;
    double dpsi;

    double du;
    double dv;
    double ddphi;
    double ddpsi;

    double u;   //m/s
    double v;   //m/s
    double phi; //rad
    double psi; //rad

    double east;  //mV
    double north; //m

    double rudder; //rad
    double sail; //rad

    double TWA; //rad
    double TWS; //m/s

    double AWA;
    double AWS;

    double u_old;   //m/s
    double v_old;   //m/s
    double phi_old; //rad
    double psi_old; //rad

    double east_old;  //m
    double north_old; //m

    //double rudder_old; //rad
    //double sail_old; //rad

    //double TWA_old; //rad
    //double TWS_old; //m/s

    double AWA_old;
    double AWS_old;

    CCubicSplineInterpolation* CsipRudderYl;
    CCubicSplineInterpolation* CsipRudderYd;
    CCubicSplineInterpolation* CsipSailYl;
    CCubicSplineInterpolation* CsipSailYd;

    double X0;
    double *F_app;
    double *F_rudder;
    double *F_sail;

    param par;
};
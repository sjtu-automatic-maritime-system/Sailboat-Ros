//
// Created by hywel on 17-5-1.
// 康梦萁 游艇设计与性能研究
// 水动力模型 YH1 阻力公式计算 YH2 Kijima模型
// 空气动力模型 IMS
// 可以改变舵角但不改变帆角

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
using namespace std;

#ifndef SAILBOAT_CDYNAMICVPP_H
#define SAILBOAT_CDYNAMICVPP_H

#endif //SAILBOAT_CDYNAMICVPP_H


class CDynamicVPP
{
public:
    // 初始：速度u，速度v，转首角速度r，坐标x0，y0，首相角Psi0，横倾角Phi0，舵角alpha_r0
    // 初始：真实风速TWS0，真实风向角TWA0
    CDynamicVPP();
    CDynamicVPP(double Psi0, double alpha_r0, double TWS0, double TWA0);
    CDynamicVPP(double u0, double v0, double r0, double X0, double Y0, double Psi0, double Phi0, double alpha_r0, double TWS0, double TWA0);
    ~CDynamicVPP();

    void init();

    void function_XH();
    void function_XR();
    void function_XS();

    void function_YH1();
    void function_YH2();
    void function_YR();
    void function_YR_old();
    void function_YS();

    void function_alpha_r();

    void function_NH();
    void function_NR();
    void function_NR_old();
    void function_NS();

    void function_cycle();

    void run(double time);

private:
    double t;
    double delta_t; //时间步长delta_t

    double mass;
    double mx;
    double my;
    double Izz;
    double KZL;
    double Cb;
    double draft;
    double Breadth;
    double Length;
    double Lwl;
    double Bwl;
    double Cru;
    double Crl;
    double Cku;
    double Ckl;
    double Swc;
    double Sr;
    double Sk;
    double Hr;
    double Hk;
    double Rho;
    double Rho_a;
    double nu;
    double A_j, A_m, A, I_j, J_j, B_j, B_m, M_i, M_d;

    double x_yh;
    double x_yr;
    double x_ys;
    double Jzz;
    double TWS;
    double TWA;  //单位角度
    double TWAO;
    double AWS;
    double AWA;
    //double TCS; //流速
    //double TCAO;//流向角
    //double TCA;

    //double ACS;
    //double ACA;
    double BS;
    //double Theta_a;
    double Beta;
    //double epsilon_a;
    //double epsilon_h;
    double alpha_r;

    double u;
    double v;
    double r;
    double u_dot;
    double v_dot;
    double r_dot;
    double Psi;
    double Phi;
    double Xo;
    double Yo;
    double Psi_dot;
    double Xo_dot;
    double Yo_dot;

    double XH;
    double XR;
    double XS;
    double YH;
    double YR;
    double YK;
    double YS;
    double NH;
    double NR;
    double NS;
};
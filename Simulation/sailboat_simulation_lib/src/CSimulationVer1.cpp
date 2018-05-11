//
// Created by hywel on 17-4-25.
//

#include "sailboat_simulation_lib/CSimulationVer1.h"

//MatrixXd v_in;
//MatrixXd g_eta;
//MatrixXd D_nu;
//MatrixXd tau_r;
//MatrixXd tau_s;


CSimulationVer1::CSimulationVer1() {

    dataShow = false;

    // --- parinit---
    par = new param();

    par->g=9.81;
    par->rou = 1025;
    par->roul = 1.29;

    par->m = 15.3;
    par->Ix = 2.002;
    par->Iz = 1.535;
    par->Ixz = -0.226;
    par->x_g = -0.051;
    par->y_g = 0;
    par->z_g = 0.069;
    par->a11 = 1.825;
    par->a22 = 35.568;
    par->a24 = -0.083;
    par->a26 = -2.784;
    par->a44 = 0.788;
    par->a46 = -0.013;
    par->a66 = 3.692;
    par->GMt = 0.215;
    //keel
    par->Ak = 0.0405;
    par->x_k = 0;
    par->y_k = 0;
    par->z_k = 0.147;
    //hull
    par->Ah = 0.314;
    par->x_h = -0.051;
    par->y_h = 0;
    par->z_h = 0.026;
    par->k1 = 35.72;
    par->k2 = 17.86;
    //Rudder
    par->Ar = 0.0158;
    par->x_r = -0.629;
    par->y_r = 0;
    par->z_r = 0.084;
    //Sail
    //Main Sail
    par->Asm = 0.59;
    par->pm1 = 0.159;
    par->pm2 = 0.219;
    par->z_sm = -1.032;
    //Fore Sail
    par->Asf = 0.54;
    par->pf1 = 0.741;
    par->pf2 = 0.369;
    par->z_sf = -0.913;



    rudderAngle = 0;
    sailAngle = 0;
    windDirection = 0;
    windVelocity = 0;
    delta_r = d2r(rudderAngle);
    delta_s = d2r(sailAngle);

    g_eta = MatrixXd::Zero(4,1);
    D_nu = MatrixXd::Zero(4,1);
    tau_r = MatrixXd::Zero(4,1);
    tau_s = MatrixXd::Zero(4,1);
    F = MatrixXd::Zero(4,1);

    M_RB = MatrixXd::Zero(4,4);
    C_RB = MatrixXd::Zero(4,4);
    M_A = MatrixXd::Zero(4,4);
    C_A = MatrixXd::Zero(4,4);
    J = MatrixXd::Zero(4,4);

    nu_dot = MatrixXd::Zero(4,1);
    eta_dot = MatrixXd::Zero(4,1);

    //v_in = MatrixXd::Zero(8,1); //没有用到，被nu和eta代替
    nu = MatrixXd::Zero(4,1);
    eta = MatrixXd::Zero(4,1);
    //v_out = MatrixXd::Zero(8,1);//没有用到
    uu = nu(0,0);
    vv = nu(1,0);
    pp = nu(2,0);
    rr = nu(3,0);
    XX = eta(0,0);
    YY = eta(1,0);
    phi = eta(2,0);
    psi = eta(3,0);

    KeelCoefInit();
    HullCoefInit();
    RudderCoefInit();
    SailCoefInit();

    Init();

    cout<<"init success"<<endl;
}



CSimulationVer1::~CSimulationVer1() {
    delete [] CsipKeelYl;
    delete [] CsipKeelYd;
    delete [] CsipHullYl;
    delete [] CsipHullYd;
    delete [] CsipRudderYl;
    delete [] CsipRudderYd;
    delete [] CsipSailYl;
    delete [] CsipSailYd;

}


void CSimulationVer1::SettingAttitudeInit(double u, double v, double p, double r, double x, double y, double ph, double ps){
    nu(0,0) = u;
    nu(1,0) = v;
    nu(2,0) = p;
    nu(3,0) = r;
    eta(0,0) = x;
    eta(1,0) = y;
    eta(2,0) = ph;
    eta(3,0) = ps;

    uu = nu(0,0);
    vv = nu(1,0);
    pp = nu(2,0);
    rr = nu(3,0);
    XX = eta(0,0);
    YY = eta(1,0);
    phi = eta(2,0);
    psi = eta(3,0);
}

//ros相关
void CSimulationVer1::Init() {


}

void CSimulationVer1::KeelCoefInit() {
    //lookup table
    double xdata[37];
    //*0.8
    double yldata[37]= {0, 0.2013, 0.4489, 0.7977, 1.003, 0.7512, 0.5264, 0.5447,
                            0.578, 0.6124, 0.6461, 0.6782, 0.7037, 0.7246, 0.7381, 0.7466, 0.7467, 0.7424,
                            0.7317, 0.7128, 0.6872, 0.656, 0.6206, 0.5822, 0.542, 0.5014, 0.4616,
                            0.4239, 0.387, 0.3486, 0.309, 0.2682, 0.2262, 0.183, 0.1389, 0.0937, 0.0476};
    double yddata[37] = {0.0049, 0.0053, 0.0095, 0.0153, 0.0259, 0.1171, 0.1544, 0.1834,
                         0.2105, 0.2401, 0.27, 0.2998, 0.3313, 0.3617, 0.392, 0.4214, 0.4478,
                         0.4734, 0.5071, 0.5454, 0.5881, 0.635, 0.6861, 0.7413, 0.8004, 0.8633,
                         0.9298, 1.0, 1.077, 1.165, 1.263, 1.369, 1.483, 1.604, 1.731, 1.863, 2.0};
    for (int i = 0; i < 37 ; i++) {
        xdata[i]=i*2.5/180*PI;
    }

    CsipKeelYl = new CCubicSplineInterpolation(xdata,yldata,37);
    CsipKeelYd = new CCubicSplineInterpolation(xdata,yddata,37);
}

void CSimulationVer1::HullCoefInit() {
    //lookup table
    double xdata[37];
    //0.3*
    double yldata[37]= {0, 0.1541, 0.273, 0.544, 0.7884, 0.929, 1.043, 0.8797, 0.4805,
                            0.5259, 0.5735, 0.6195, 0.66, 0.6923, 0.7183, 0.7345, 0.7445, 0.745,
                            0.7383, 0.7271, 0.7125, 0.6945, 0.673, 0.6481, 0.6196, 0.5877, 0.5522,
                            0.5132, 0.4707, 0.4246, 0.3748, 0.3215, 0.2645, 0.2039, 0.1396, 0.0717, 0};
    double yddata[37] = {0.0055, 0.0057, 0.0075, 0.0126, 0.0184, 0.0281, 0.0487, 0.1113,
                         0.1904, 0.2225, 0.2566, 0.2897, 0.3251, 0.3612, 0.3971, 0.4307, 0.465,
                         0.4935, 0.5206, 0.5583, 0.6006, 0.6478, 0.6999, 0.757, 0.8192, 0.8866,
                         0.9593, 1.037, 1.121, 1.21, 1.305, 1.406, 1.513, 1.625, 1.744, 1.869, 2.0};
    for (int i = 0; i < 37 ; i++) {
        xdata[i]=i*2.5/180*PI;
    }

    CsipHullYl = new CCubicSplineInterpolation(xdata,yldata,37);
    CsipHullYd = new CCubicSplineInterpolation(xdata,yddata,37);
}

void CSimulationVer1::RudderCoefInit() {
    //lookup table
    double xdata[37];
    double yldata[37]= {0, 0.3212, 0.683, 0.8553, 0.9999, 1.095, 0.652, 0.6738, 0.7474,
                        0.8103, 0.8848, 0.9566, 1.024, 1.085, 1.14, 1.186, 1.222, 1.248, 1.018,
                        0.7698, 0.7468, 0.7174, 0.6837, 0.6453, 0.6056, 0.5646, 0.5223, 0.4784,
                        0.4329, 0.3858, 0.3368, 0.286, 0.2331, 0.1782, 0.1211, 0.0617, 0};
    double yddata[37] = {0.0072, 0.009, 0.012, 0.0161, 0.0224, 0.0324, 0.1509, 0.1952,
                         0.2271, 0.2624, 0.2917, 0.3198, 0.3469, 0.3721, 0.3962, 0.418, 0.4338,
                         0.4466, 0.5092, 0.576, 0.5956, 0.6071, 0.6147, 0.6325, 0.663, 0.7062,
                         0.7621, 0.8305, 0.9114, 1.005, 1.11, 1.229, 1.359, 1.501, 1.655, 1.822, 2.0};
    for (int i = 0; i < 37 ; i++) {
        xdata[i]=i*2.5/180*PI;
    }

    CsipRudderYl = new CCubicSplineInterpolation(xdata,yldata,37);
    CsipRudderYd = new CCubicSplineInterpolation(xdata,yddata,37);
}

void CSimulationVer1::SailCoefInit() {
    //lookup table
    double xdata[37];//?same with rudder
    double yldata[37]= {0, 0.2212, 0.583, 0.8553, 0.9999, 1.095, 0.652, 0.6738, 0.7474,
                        0.8103, 0.8848, 0.9566, 1.024, 1.085, 1.14, 1.186, 1.222, 1.248, 1.018,
                        0.7698, 0.7468, 0.7174, 0.6837, 0.6453, 0.6056, 0.5646, 0.5223, 0.4784,
                        0.4329, 0.3858, 0.3368, 0.286, 0.2331, 0.1782, 0.1211, 0.0617, 0};
    double yddata[37] = {0.0072, 0.009, 0.012, 0.0161, 0.0224, 0.0324, 0.1509, 0.1952,
                         0.2271, 0.2624, 0.2917, 0.3198, 0.3469, 0.3721, 0.3962, 0.418, 0.4338,
                         0.4466, 0.5092, 0.576, 0.5956, 0.6071, 0.6147, 0.6325, 0.663, 0.7062,
                         0.7621, 0.8305, 0.9114, 1.005, 1.11, 1.229, 1.359, 1.501, 1.655, 1.822, 2.0};
    for (int i = 0; i < 37 ; i++) {
        xdata[i]=i*2.5/180*PI;
    }

    CsipSailYl = new CCubicSplineInterpolation(xdata,yldata,37);
    CsipSailYd = new CCubicSplineInterpolation(xdata,yddata,37);
}

double* CSimulationVer1::KeelCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the keel
    //and compute Clk/Cdk from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    if (alpha>=0 && alpha<=PI/2)
    {
        xp = alpha;
        Clk = CsipKeelYl->GetYByX(xp);
        Cdk = CsipKeelYd->GetYByX(xp);
    } else if (alpha >= PI/2 && alpha <=PI)
    {
        xp = PI -alpha;
        Clk = -1*CsipKeelYl->GetYByX(xp);
        Cdk = CsipKeelYd->GetYByX(xp);
    } else if (alpha >= -PI/2 && alpha < 0)
    {
        xp = -alpha;
        Clk = -1*CsipKeelYl->GetYByX(xp);
        Cdk = CsipKeelYd->GetYByX(xp);
    } else
    {
        xp = alpha + PI;
        Clk = CsipKeelYl->GetYByX(xp);
        Cdk = CsipKeelYd->GetYByX(xp);
    }

    double *tmp;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}

double* CSimulationVer1::HullCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the hull
    //and compute Clh/Cdh from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    if (alpha>=0 && alpha<=PI/2)
    {
        xp = alpha;
        Clk = CsipHullYl->GetYByX(xp);
        Cdk = CsipHullYd->GetYByX(xp);
    } else if (alpha >= PI/2 && alpha <=PI)
    {
        xp = PI -alpha;
        Clk = -1*CsipHullYl->GetYByX(xp);
        Cdk = CsipHullYd->GetYByX(xp);
    } else if (alpha >= -PI/2 && alpha < 0)
    {
        xp = -alpha;
        Clk = -1*CsipHullYl->GetYByX(xp);
        Cdk = CsipHullYd->GetYByX(xp);
    } else
    {
        xp = alpha + PI;
        Clk = CsipHullYl->GetYByX(xp);
        Cdk = CsipHullYd->GetYByX(xp);
    }

    double *tmp ;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}

double* CSimulationVer1::RudderCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the rudder
    //and compute Clr/Cdr from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    if (alpha>=0 && alpha<=PI/2)
    {

        xp = alpha;

        Clk = CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    } else if (alpha >= PI/2 && alpha <=PI)
    {
        xp = PI -alpha;
        Clk = -CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    } else if (alpha >= -PI/2 && alpha < 0)
    {
        xp = -alpha;
        Clk = -CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    } else
    {
        xp = alpha + PI;
        Clk = CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    }

    double *tmp;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}

double* CSimulationVer1::SailCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the rudder
    //and compute Clr/Cdr from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    if (alpha>=0 && alpha<=PI/2)
    {

        xp = alpha;

        Clk = CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    } else if (alpha >= PI/2 && alpha <=PI)
    {
        xp = PI -alpha;
        Clk = -CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    } else if (alpha >= -PI/2 && alpha < 0)
    {
        xp = -alpha;
        Clk = -CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    } else
    {
        xp = alpha + PI;
        Clk = CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    }

    double *tmp;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}


void CSimulationVer1::RestoreForce() {
    //可以不考虑恢复力矩
    g_eta(2,0) = par->m*par->g*par->GMt*sin(phi)*cos(phi);

    //cout<<"g_eta "<<endl<<g_eta<<endl;
}

void CSimulationVer1::DampingForce() {
    double phi_dot = pp;
    //double psi_dot = rr;
    //不考虑横摇的影响
    double psi_dot = rr*cos(phi);
    //translation damping

    double* D_keel_tmp;
    double* D_hull_tmp;

    D_keel_tmp = Damping(par->rou,par->Ak,par->x_k,par->y_k,par->z_k,0);

    D_hull_tmp = Damping(par->rou,par->Ah,par->x_h,par->y_h,par->z_h,1);

    D_nu(0,0) = D_keel_tmp[0]+D_hull_tmp[0]+0;
    D_nu(1,0) = D_keel_tmp[1]+D_hull_tmp[1]+0;
    //D_nu(2,0) = 0;
    //D_nu(3,0) = D_keel_tmp[3]+D_hull_tmp[3]+par->k2 * psi_dot * fabs(psi_dot);
    //不考虑横摇的影响
    D_nu(2,0) = D_keel_tmp[2]+D_hull_tmp[2]+par->k1 * phi_dot * fabs(phi_dot);
    D_nu(3,0) = D_keel_tmp[3]+D_hull_tmp[3]+par->k2 * psi_dot * fabs(psi_dot) * cos(phi);

    //cout<<"D_keel_tmp"<<endl<<D_keel_tmp[0]<<endl<<D_keel_tmp[1]<<endl<<D_keel_tmp[2]<<endl<<D_keel_tmp[3]<<endl;
    //cout<<"D_hull_tmp"<<endl<<D_hull_tmp[0]<<endl<<D_hull_tmp[1]<<endl<<D_hull_tmp[2]<<endl<<D_hull_tmp[3]<<endl;
    //cout<<"D_nu "<<endl<<D_nu<<endl;
    delete [] D_hull_tmp;
    delete [] D_keel_tmp;
}

double* CSimulationVer1::Damping(double rou, double A, double x, double y, double z, int id) {
    double v_au = -uu + rr*y;
    //double v_av = -vv; //why
    //不考虑横摇的影响
    double v_av = -vv - rr*x + pp*z; //why

    //    double alphaTmp;
    //    if(fabs(v_au)< 0.0001 && fabs(v_av)< 0.0001)
    //        alphaTmp = 0;
    //    else
    double alphaTmp = atan2(-v_av,-v_au);

    double V2 = pow(v_au,2)+pow(v_av,2);

    double* cld;

    if (id == 0)
    {
        cld = KeelCoef(alphaTmp);
    } else if (id == 1)
    {
        cld = HullCoef(alphaTmp);
    }

    double k1 = 0.8;
    double k2 = 1;
    double L = k1*0.5*rou*A*V2*cld[0];
    double D = k2*0.5*rou*A*V2*cld[1];
    //cout<<"L = "<<L<<endl;
    //cout<<"D = "<<D<<endl;

    double tmp1 = -L*sin(alphaTmp)+D*cos(alphaTmp);
    double tmp2 =  L*cos(alphaTmp)+D*sin(alphaTmp);

    double* D_tmp;
    D_tmp = new double[4];

    D_tmp[0] = tmp1;
    D_tmp[1] = tmp2;
    D_tmp[2] = -tmp2*z;  //why?
    D_tmp[3] = tmp2*x;

    delete [] cld;

    return D_tmp;

    //cout<<"Dh_tmp"<<endl<<Dh_tmp[0]<<endl;
    //return Dh_tmp;
}
void CSimulationVer1::ControlForceRudder() {
    double v_au = -uu + rr * par->y_r;
    //double v_av = -vv - rr * par->x_r;
    //不考虑横摇的影响
    double v_av = -vv - rr * par->x_r + pp*par->z_r;
    double beta;
    if(fabs(v_au)< 0.001 && fabs(v_av) < 0.001)
        beta = 0;
    else
        beta = atan2(-v_av,-v_au);

    float alphaTmp;
    alphaTmp = delta_r + beta;

    alphaTmp = limitsPi(alphaTmp);

    double V2 = pow(v_au,2) + pow(v_av,2);
    double *cld;
    cld = RudderCoef(alphaTmp);

    double k1 = 0.8;
    double k2 = 0.8;//rudder effective coefficient
    double L,D;
    L = k1*0.5*par->rou*par->Ar*V2*cld[0];
    D = k2*0.5*par->rou*par->Ar*V2*cld[1];

    double tmp1 = L*sin(alphaTmp)-D*cos(alphaTmp);
    double tmp2 = -L*cos(alphaTmp)-D*sin(alphaTmp);
    double tmp3 = tmp1*cos(delta_r)+tmp2*sin(delta_r);
    double tmp4 = -tmp1*sin(delta_r)+tmp2*cos(delta_r);
    tau_r(0,0) = tmp3;
    tau_r(1,0) = tmp4;
    //tau_r(2,0) = 0;
    tau_r(2,0) = -tmp4*par->z_r;
    tau_r(3,0) = tmp4*par->x_r;

    delete [] cld;

    //cout<<"delta_r = "<<delta_r<<endl;

    //cout<<"tau_r"<<endl<<tau_r<<endl;
}



void CSimulationVer1::ControlForceSail() {
    MatrixXd J1(2,2);
    J1(0,0) = cos(psi);
    J1(0,1) = sin(psi);
    //J1(1,0) = -sin(psi);
    //J1(1,1) = cos(psi);
    //不考虑横摇的影响
    J1(1,0) = -cos(phi)*sin(psi);
    J1(1,1) = cos(phi)*cos(psi);

    MatrixXd v_twn(2,1),v_twb(2,1);
    v_twn(0,0) = windVelocity*cos(windDirection);
    v_twn(1,0) = windVelocity*sin(windDirection);
    v_twb =J1*v_twn;

    double x_s = par->pm1-par->pm2*cos(delta_s);
    double y_s = par->pm2*sin(delta_s);
    double z_s = par->z_sm;

    double k1 = 0.8;
    double k2 = 0.6;//mail & fore sail effective coeff

    //    double  v_au = uu;
    //    double  v_av = vv;
    //不清楚
    double v_au = uu - rr*y_s;
    //double v_av = vv + rr*x_s;
    //不考虑横摇的影响
    double v_av = vv + rr*x_s - pp*z_s;

    MatrixXd v_as(2,1),v_aw(2,1);
    v_as(0,0) = v_au;
    v_as(1,0) = v_av;
    v_aw = v_twb - v_as;


    if(fabs(v_aw(1,0)) < 0.001 && fabs(v_aw(0,0)) < 0.001 )
        AWA = 0;
    else
        AWA = atan2(-v_aw(1,0),-v_aw(0,0));

    double alphaTmp = delta_s + AWA;

    alphaTmp = limitsPi(alphaTmp);

    double V2 = pow(v_aw(0,0),2)+pow(v_aw(1,0),2);

    AWS = pow(V2,0.5);

    double *cld;
    cld = SailCoef(alphaTmp);

    double L = 0.5*par->roul*par->Asm*V2*cld[0];
    double D = 0.5*par->roul*par->Asm*V2*cld[1];
    double tmp1 = L*sin(alphaTmp)-D*cos(alphaTmp);
    double tmp2 = -L*cos(alphaTmp)-D*sin(alphaTmp);
    double tmp3 = tmp1*cos(delta_s)+tmp2*sin(delta_s);
    double tmp4 = -tmp1*sin(delta_s)+tmp2*cos(delta_s);
    tau_s(0,0) = tmp3;
    tau_s(1,0) = tmp4;
    //tau_s(2,0) = 0;
    tau_s(2,0) = -tmp4*z_s;
    tau_s(3,0) = tmp4*x_s;

    delete [] cld;
    //cout<<"tau_s"<<endl<<tau_s<<endl;
}

void CSimulationVer1::Equation() {
    //Rigid body Inertia Matrix & Coriolis-centripetal Matrix
//    M_RB << par->m,      0,    0,               0,
//            0,      par->m,    0, par->m*par->x_g,
//            0,           0,  0.5,               0,
//            0,par->m*par->x_g, 0,         par->Iz;
//    C_RB << 0,           0,    0,-par->m*(par->x_g*rr+vv),
//            0,           0,    0,       par->m*uu,
//            0,           0,  0.5,               0,
//            par->m*(par->x_g*rr+vv),-par->m*uu,0,0;
    //不考虑横摇影响
    M_RB << par->m,0,0,0,
            0,par->m,-par->m*par->z_g,par->m*par->x_g,
            0,-par->m*par->z_g,par->Ix,-par->Ixz,
            0,par->m*par->x_g,-par->Ixz,par->Iz;
    C_RB << 0,0,par->m*par->z_g*rr,-par->m*(par->x_g*rr+vv),
            0,0,0,par->m*uu,
            -par->m*par->z_g*rr,0,0,0,
            par->m*(par->x_g*rr+vv),-par->m*uu,0,0;

    //Added mass Inertia Matrix & Coriolis-centripetal Matrix
//    M_A << par->a11,0,0,0,
//            0,par->a22,0,par->a26,
//            0,0,0.5,0,
//            0,par->a26,0,par->a66;
//    C_A << 0,0,0,-(par->a22*vv+par->a26*rr),
//            0,0,0,par->a11*uu,
//            0,0,0.5,0,
//            par->a22*vv+par->a26*rr,-par->a11*uu,0,0;
    //不考虑横摇影响
    M_A << par->a11,0,0,0,
            0,par->a22,par->a24,par->a26,
            0,par->a24,par->a44,par->a46,
            0,par->a26,par->a46,par->a66;
//    C_A << 0,0,0,-(par->a22*vv+par->a26*rr),
//            0,0,0,par->a11*uu,
//            0,0,0,0,
//            par->a22*vv+par->a26*rr,-par->a11*uu,0,0;
    //不考虑横摇影响
    C_A << 0,0,0,-(par->a22*vv+par->a24*pp+par->a26*rr),
            0,0,0,par->a11*uu,
            0,0,0,0,
            par->a22*vv+par->a24*pp+par->a26*rr,-par->a11*uu,0,0;
    MatrixXd M(4,4),C(4,4);
    M = M_RB + M_A;
    C = C_RB + C_A;
//    J << cos(psi),-sin(psi),0,0,
//            sin(psi),cos(psi),0,0,
//            0,0,1,0,
//            0,0,0,1;
    //不考虑横摇影响
    J << cos(psi),-sin(psi)*cos(phi),0,0,
            sin(psi),cos(psi)*cos(phi),0,0,
            0,0,1,0,
            0,0,0,cos(phi);

    //cout<<"M="<<endl<<M<<endl;
    //cout<<"C="<<endl<<C<<endl;
    //cout<<"J="<<endl<<J<<endl;

    F_input = tau_r + tau_s;

    F = -g_eta - D_nu + tau_r + tau_s;
    nu_dot = M.inverse()*(-C*nu*0 + F);
    eta_dot = J*nu;
}


void CSimulationVer1::Sailboat_Test(double time, double d_t) {
    double delta_t = d_t;
    //ROS_INFO("Sailboat_Test start");
    for (double t = 0; t < time ; t = t+delta_t) {
        Sailboat_Calc(delta_t);
    }
}

void CSimulationVer1::Sailboat_Calc(double d_t) {

    double delta_t = d_t;
    RestoreForce();
    DampingForce();
    ControlForceRudder();
    ControlForceSail();

    Equation();

    nu = nu + nu_dot * delta_t;
    eta = eta + eta_dot * delta_t;

    // 数据处理
    // 横摇相关归零
    // 限幅
    //nu(2,0) = 0;
    //eta(2,0) = 0;
    nu(3,0) = limitsPi(nu(3,0));
    eta(3,0) = limitsPi(eta(3,0));

    if(nu(2,0) > PI/6)
        nu(2,0) = PI/6;
    else if (nu(2,0) < -PI/6)
        nu(2,0) = -PI/6;
    if(eta(2,0) > PI/6)
        eta(2,0) = PI/6;
    else if (eta(2,0) < -PI/6)
        eta(2,0) = -PI/6;


    if (dataShow == true)
    {
        cout<<"F="<<endl<<F<<endl;
        cout<<"nu="<<endl<<nu<<endl;
        cout<<"eta="<<endl<<eta<<endl;
        cout<<"----------------"<<endl;
    }

    uu = nu(0,0);
    vv = nu(1,0);
    pp = nu(2,0);
    rr = nu(3,0);
    XX = eta(0,0);
    YY = eta(1,0);
    phi = eta(2,0);
    psi = eta(3,0);

}


void CSimulationVer1::ShowData()
{
    dataShow = true;
}

void CSimulationVer1::HideData()
{
    dataShow = false;
}

double CSimulationVer1::d2r(double d) {
    return d*PI/180;
}

double CSimulationVer1::r2d(double r) {
    return r*180/PI;
}

double CSimulationVer1::limitsPi(double tmp)
{
    while (fabs(tmp)>PI)
    {
        if(tmp>0)
            tmp = tmp-2*PI;
        else
            tmp = tmp+2*PI;
    }
    return tmp;
}



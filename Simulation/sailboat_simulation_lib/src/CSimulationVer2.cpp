//
// Created by hywel on 17-5-4.
//

#include "sailboat_simulation_lib/CSimulationVer2.h"

param::param() {
    g=9.801;
    rouwater=1025;
    rouair=1.22;
    // hydrostatics
    m=15.3;
    mx=2.02356;
    my=12.53222;
    mz=39.71;
    Ixx=1.299;
    Iyy=1.63;
    Izz=1.262;
    Jxx=0.16;
    Jyy=1.628;
    Jzz=0.798;
    lwl=1.3113;
    draft=0.069;
    GMt=0.215;
    areawethull=0.53;
    draftwithkeel=0.4331;
    // hull & keel
    qf=0.5*rouwater*lwl*draft;
    qh=0.5*rouwater*lwl*draftwithkeel;
    qh2=0.5*rouwater*lwl*pow(draftwithkeel,2);
    qh3=0.5*rouwater*pow(lwl,2)*draftwithkeel;
    Xvdpsi=-1.91e-2;
    Ydfi=0.219;
    Ydpsi=-4.01e-3;
    Kdfi=-0.353;
    Ndpsi=-5.89e-3;
    Xvv=3.38e-1*qh;
    Xfifi=1.40e-3*qh;
    Xvvvv=-1.84*qh;
    Yv=-5.35e-1*qh;
    Yfi=-5.89e-3*qh;
    Yvfifi=7.37e-1*qh;
    Yvvfi=-5.53e-1*qh;
    Yvvv=3.07*qh;
    Kv=2.8e-1*qh2;
    Kfi=3.36e-3*qh2;
    Kvfifi=-4.07e-1*qh2;
    Kvvfi=2.24e-1*qh2;
    Kvvv=-1.38*qh2;
    Nv=-3.23e-2*qh3;
    Nfi=-1.52e-2*qh3;
    Nvfifi=2.71e-4*qh3;
    Nvvfi=-9.06e-2*qh3;
    Nvvv=-2.98e-2*qh3;
    // sail
    areasail=1.5*0.375;
    msail=0.8;
    sail_xcg=0.91-1.5/2+0.051;
    sail_ycg=0;
    sail_zcg=1.5/2+0.04+(0.4331-0.18-0.069);
    sail_xgce=0.91-1.5/2+0.051;
    sail_ygce=0;
    sail_zgce=-(1.5/2+0.04+(0.4331-0.18-0.069));
    // rudder
    arearudder=0.0162;
    rudder_xgce=-0.629+0.051;
    rudder_ygce=0;
    rudder_zgce=0.084-0.069;
    Cxrudder=-3.79e-2*qh;
    Cyrudder=-1.8e-1*qh;
    Ckrudder=9.76e-2*qh2;
    Cnrudder=9.74e-2*qh3;
    //keel
    areakeel=0.0405;
    keel_xgce=-0.051;
    keel_ygce=0;
    keel_zgce=0.147-0.069;
}

param::~param() {}


CSimulationVer2::CSimulationVer2() {

    u_old = 0;   //m/s
    v_old = 0;   //m/s
    phi_old = 0; //rad
    psi_old = 0; //rad

    east_old = 0;  //m
    north_old = 0; //m

    AWA_old = 0;
    AWS_old = 0;

    RudderCoefInit();
    SailCoefInit();

}

CSimulationVer2::~CSimulationVer2() {

    delete [] CsipRudderYl;
    delete [] CsipRudderYd;
    delete [] CsipSailYl;
    delete [] CsipSailYd;
    delete [] F_app;
    delete [] F_rudder;
    delete [] F_sail;
}

void CSimulationVer2::CalcAWSAWA() {
    double leeway = atan2(v_old , u_old);
    double Vyacht = sqrt(u_old*u_old + v_old*v_old);
    if ((pow(Vyacht,2)*pow(cos(AWA_old - leeway),2) +TWS*TWS - Vyacht*Vyacht)<0)
        AWS =  sqrt(2*Vyacht*TWS*cos(AWA_old - leeway) +TWS*TWS + Vyacht*Vyacht);
    else
        AWS = Vyacht*cos(AWA_old - leeway) + sqrt(Vyacht*Vyacht*pow(cos(AWA_old - leeway),2) +TWS*TWS - Vyacht*Vyacht);
    //the wind directi is defined as common,the wind of north means the windangle is 0
    TWA = limitsPi(TWA);
    //now the windangle[-pi,pi) and is defined as regular
    //psi_now(-pi,pi)
    double twa_cal_body=TWA-psi_old;
    twa_cal_body = limitsPi(twa_cal_body);
    AWA = twa_cal_body - asin(Vyacht*sin(twa_cal_body - leeway)/AWS);
}

double CSimulationVer2::X004() {

    double Vyacht = sqrt(u_old*u_old + v_old*v_old);
    double X0 =-1*resist0H(Vyacht);

    return X0;
}

double* CSimulationVer2::FM_add() {
    double* tmpAdd;
    tmpAdd = new double[4];
    tmpAdd[0] = 0;
    tmpAdd[1] = 0;
    tmpAdd[2] = 0;
    tmpAdd[3] = 0;
    double leeway = atan2(v_old , u_old);
    double Vyacht = sqrt(u_old*u_old + v_old*v_old);

    double q=0.5*par.rouwater*pow(Vyacht,2)*(par.areawethull*0.7/2+par.areakeel);
    //似乎因横倾产生的增阻乘的是船体湿表面积，诱导阻力和因leeway产生的增阻才乘area_LT侧向面积？此处为方便计做了任！性！简化
    //sail(angle) is positive when the sail trailing is on the starboard(righthand)(-pi,pi)

    double Fn = Vyacht/sqrt(par.g*par.lwl);
    double sideForce_TH = q*cos(phi_old)*coefLTH( phi_old , leeway , Fn );
    double resistAdd_TH = -1*q*coefRTH( phi_old , leeway , Fn );
    double sideForce_Y = sideForce_TH*cos(leeway) + (-1)*fabs(resistAdd_TH)*sin(leeway);
    double resistAdd_X = resistAdd_TH*cos(leeway);

    tmpAdd[0] = resistAdd_X;
    tmpAdd[1] = sideForce_Y;
    tmpAdd[2] = sideForce_Y/cos(phi_old)*par.draft;
    // 在K的计算上其实有些混乱，对rudder而言，产生的K作用对应的转动中心点在keel顶端（也就是本船默认的重心位置）上；对帆以及此处的船体&龙骨，其产生的K为保持一致性，进而都让其转动中心点在水线面上
    //假设此处的船体&龙骨侧向力的作用点位于keel顶端（也就是本船默认的重心位置）上（这与我的VPP中的假设是一致的）
    tmpAdd[3] = 0;

    return tmpAdd;
}



double* CSimulationVer2::FM_rudder() {
    //rudder is positive when operate the right rudder(-pi,pi)
    double* tmpRudder;
    tmpRudder = new double[4];

    double leeway = atan2(v_old , u_old);
    double Vyacht = sqrt(u_old*u_old + v_old*v_old);
    double q=0.5*par.rouwater*pow(Vyacht,2)*par.arearudder;
    //sail(angle) is positive when the sail trailing is on the starboard(righthand)(-pi,pi)
    double attack_rudder=leeway+rudder;
    double* cld;

    cld=RudderCoef(attack_rudder);

    double Cx=cld[0]*sin(leeway)-cld[1]*cos(leeway);
    double Cy=-cld[0]*cos(leeway)-cld[2]*sin(leeway);

    tmpRudder[0]=Cx*pow(cos(phi_old),2)*q;
    tmpRudder[1]=Cy*pow(cos(phi_old),2)*q;
    tmpRudder[2]=-q*Cy*par.rudder_zgce;
    tmpRudder[3]=(q*Cy*par.rudder_xgce+q*Cx*par.rudder_zgce*cos(phi_old)*sin(phi_old)+q*Cx*par.rudder_ygce*pow(sin(phi_old),2))*pow(cos(phi_old),2)*2;
    // 此处的系数2是我人为添加的！！！！！

    delete [] cld;
    return tmpRudder;
}


double* CSimulationVer2::FM_sail() {
    //awa is positive when the wind blow to the right face of the sail(-pi,pi)

    double* tmpSail;
    tmpSail = new double[4];
    double leeway = atan2(v_old , u_old);
    double Vyacht = sqrt(u_old*u_old + v_old*v_old);
    double q=0.5*par.rouair*pow(AWS,2)*par.areasail;
    //sail(angle) is positive when the sail trailing is on the starboard(righthand)(-pi,pi)
    double attack_sail=AWA+sail;

    double* cld;
    cld = SailCoef(attack_sail);

    double Cx=cld[0]*sin(AWA)-cld[1]*cos(AWA);
    double Cy=-cld[0]*cos(AWA)-cld[1]*sin(AWA);

    tmpSail[0]=Cx*pow(cos(phi_old),2)*q;
    tmpSail[1]=Cy*pow(cos(phi_old),2)*q;
    tmpSail[2]=-tmpSail[1]*par.sail_zgce/cos(phi_old);
    tmpSail[3]=tmpSail[1]*par.sail_xgce + tmpSail[0]*par.sail_zgce*sin(phi_old);

    delete [] cld;
    return tmpSail;

}


double CSimulationVer2::coefLTH(double fi, double beta, double Fn) {
    double attackHull = fabs(beta*cos(fi));
    double coef = -0.07092*pow(fi,2) + 1.649*(1 - 0.2321*pow(fi,2))*attackHull + 0.4534*pow(attackHull,2) - 39.31*fabs(fi)*Fn*pow(attackHull,3);
    if (beta > 0)
        coef = -coef;
    return coef;
}

double CSimulationVer2::coefRTH(double fi, double beta , double Fn) {
    beta = fabs(beta);
    double attackHull = beta*cos(fi);
    double coef_LaTH = -0.07092*pow(fi,2) + 1.649*(1 - 0.2321*pow(fi,2))*attackHull;
    double ARe = 8*PI*(1.649*(1-0.2321*pow(fi,2)))/(4*PI*PI - pow(1.649*(1-0.2321*pow(fi,2)),2));
    double coef;
    if (attackHull > 0.15)
        coef = 0.002319*fabs(fi) + 0.3501*pow(fi,2)*pow(Fn,4) + (0.8141 + 5.47*pow(Fn,2))*(pow(coef_LaTH,2)/(PI*ARe)) + 74.9*pow((attackHull - 0.15),3);
    else
        coef = 0.002319*fabs(fi) + 0.3501*pow(fi,2)*pow(Fn,4) + (0.8141 + 5.47*pow(Fn,2))*(pow(coef_LaTH,2)/(PI*ARe));
    return coef;
}

double CSimulationVer2::resist0H(double Vyacht) {
    double force = -0.04742 - 0.67109*Vyacht + 11.33722*pow(Vyacht,2) - 25.40506*pow(Vyacht,3) + 20.88081*pow(Vyacht,4) - 4.87654*pow(Vyacht,5);
    //double force = -0.04742 - 0.67109*Vyacht + 11.33722*Vyacht^2 - 25.40506*Vyacht^3 + 20.88081*Vyacht^4 - 4.87654*Vyacht^5;
    if (force < 0)
        force = 0;
    return force;

}

void CSimulationVer2::RudderCoefInit() {
    //lookup table
    double xdata[91];
    double yldata[91]= {0,      0.1095,	0.2202,	0.3401,	0.4599, 0.5796,	0.6984,	0.7936, 0.8804,
                        0.952,  0.9981, 0.6289, 0.6369,	0.6631, 0.6912, 0.7478, 0.8043, 0.8609,
                        0.9159, 0.9606, 1.0053, 1.0375, 1.0629,	1.088,  1.1114, 1.1291, 1.1398,
                        1.1399, 1.1312, 1.1086, 1.0858, 1.0589,	1.027,  0.9872, 0.9437, 0.885,
                        0.8259, 0.7664, 0.6999, 0.6313, 0.5627,	0.4755, 0.3809, 0.2952, 0.2132,
                        0.132,  0.0398, -0.06,  -0.1589,-0.2532,-0.3103,-0.3812,-0.4692,-0.5347,
                        -0.5988,-0.6611,-0.7177,-0.7738,-0.8313,-0.8835,-0.9321,-0.9655,-0.9991,
                        -1.0342,-1.0591,-1.0732,-1.0717,-1.0596,-1.0475,-1.0243,-0.986, -0.9449,
                        -0.8924,-0.8381,-0.7833,-0.7323,-0.7056,-0.6813,-0.699, -0.7302,-0.7662,
                        -0.874, -0.8646,-0.8012,-0.7206,-0.6199,-0.5167,-0.4094,-0.2838,-0.1419,0};
    double yddata[91] = {0.0083,  0.0118,   0.0164, 0.0287, 0.0418, 0.0634, 0.0851, 0.1095, 0.133,
                         0.1547,  0.1875,	0.2766,	0.2958,	0.3127,	0.3504,	0.4099,	0.4705,	0.5411,
                         0.6254,  0.7185,	0.8123,	0.9009,	0.9865,	1.0723,	1.1541,	1.2256,	1.2967,
                         1.3676,  1.4391,	1.514,  1.5884,	1.6528,	1.7052,	1.7549,	1.8031,	1.8413,
                         1.8798,  1.919,	1.9479,	1.9754,	2.0041,	2.0241,	2.0405,	2.0595,	2.0696,
                         2.079,	  2.0799,	2.076,	2.0661,	2.047,	2.0125,	1.9804,	1.9527,	1.9106,
                         1.8728,  1.843,	1.8091,	1.7703,	1.7194,	1.6718,	1.626,  1.5595,	1.4931,
                         1.4277,  1.3667,	1.3105,	1.24,	1.1589,	1.0771,	0.9937,	0.908,  0.8226,
                         0.7412,  0.661,	0.5817,	0.5076,	0.4629,	0.4185,	0.3895,	0.3661,	0.3378,
                         0.293,	  0.2487,	0.2035,	0.1582,	0.1156,	0.0794,	0.0543,	0.0353,	0.0216, 0.008};
    for (int i = 0; i < 91 ; i++) {
        xdata[i]=i*2/180*PI;
    }

    CsipRudderYl = new CCubicSplineInterpolation(xdata,yldata,91);
    CsipRudderYd = new CCubicSplineInterpolation(xdata,yddata,91);
}

void CSimulationVer2::SailCoefInit() {
    //lookup table
    double xdata[91];//?same with rudder
    double yldata[91]= {0,      0.1465,	   0.293,   0.4245,	  0.509,  0.6681,  0.4762,  0.1162, 0.1923,
                        0.2745,	0.3587,	  0.4448,	0.5312,	  0.6177, 0.7042,  0.7923,	0.8384,	0.8573,
                        0.9022,	0.947,	  0.9856,	1.0065,	  1.0275, 1.0388,  1.0447,	1.0487,	1.0364,
                        1.024,	1.0014,	  0.9737,	0.9439,	  0.9093, 0.8747,  0.8233,	0.772,	0.7147,
                        0.6557,	0.5967,	  0.5378,	0.4778,	  0.4162, 0.3547,  0.2931,	0.2315,	0.1674,
                        0.1032,	0.039,	  -0.0252,	-0.0878, -0.1494,-0.211,  -0.2726, -0.3335,-0.3925,
                        -0.4515,-0.5079,  -0.5643,	-0.6087, -0.6503,-0.6907, -0.7299, -0.7691,-0.8083,
                        -0.8476,-0.8824,  -0.9169,  -0.9430, -0.9640,-0.9730, -0.9432, -0.9134,-0.8743,
                        -0.8279,-0.7814,  -0.7423,	-0.7056, -0.6696,-0.6561, -0.6425, -0.6434,-0.6622,
                        -0.684,	 -0.7645, -0.845,	-0.7892, -0.7245,-0.6592, -0.4944, -0.3296,-0.1648,0};
    double yddata[91] = {0.0147,    0.0168, 0.0225, 0.0313,	0.0398,	0.0569,	0.0699,	0.1496,	0.1823,
                         0.2178,	0.2563,	0.2971,	0.3414,	0.389,	0.439,	0.4929,	0.5591,	0.6317,
                         0.7011,	0.7705,	0.84,	0.91,   0.9801,	1.045,	1.1072,	1.1686,	1.2242,
                         1.2797,	1.3321,	1.3831,	1.4334,	1.4823,	1.5311,	1.5714,	1.6116,	1.6473,
                         1.6816,	1.7115,	1.738,	1.7608,	1.777,	1.7916,	1.7979,	1.8041,	1.8026,
                         1.801,	    1.7932,	1.7838,	1.7725,	1.76,	1.7441,	1.7238,	1.7022,	1.6763,
                         1.6503,	1.6188,	1.5873,	1.5527,	1.5174,	1.4756,	1.4281,	1.3805,	1.333,
                         1.2854,	1.3734,	1.3037,	1.1835,	1.0708,	0.9855,	0.9177,	0.8499,	0.7805,
                         0.71,	    0.6394,	0.5761,	0.5151,	0.4546,	0.4144,	0.3743,	0.3362,	0.3006,
                         0.2651,	0.2323,	0.1995,	0.163,	0.1263,	0.0896,	0.0734,	0.0573,	0.0411,	0.025};
    for (int i = 0; i < 91 ; i++) {
        xdata[i]=i*2/180*PI;
    }

    CsipSailYl = new CCubicSplineInterpolation(xdata,yldata,91);
    CsipSailYd = new CCubicSplineInterpolation(xdata,yddata,91);
}

double* CSimulationVer2::RudderCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the rudder
    //and compute Clr/Cdr from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    alpha = limitsPi(alpha);

    if (alpha>=0)
    {

        xp = alpha;
        Clk = CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    }
    else
    {
        xp = -alpha;
        Clk = -1*CsipRudderYl->GetYByX(xp);
        Cdk = CsipRudderYd->GetYByX(xp);
    }
    double *tmp;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}

double* CSimulationVer2::SailCoef(double alpha) {
    //generate a lookup table for the lift/drag coefficients for the rudder
    //and compute Clr/Cdr from the lookup table using interpolation.
    double xp ;
    double Clk;
    double Cdk;

    alpha = limitsPi(alpha);

    if (alpha>=0)
    {

        xp = alpha;
        Clk = CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    }
    else
    {
        xp = -alpha;
        Clk = -1*CsipSailYl->GetYByX(xp);
        Cdk = CsipSailYd->GetYByX(xp);
    }

    double *tmp;
    tmp = new double[2];
    tmp[0] = Clk;
    tmp[1] = Cdk;
    return tmp;
}

void CSimulationVer2::FourD0F() {
    //horizontal body axis system, (x,y,z,U,V,W,fi_cap=phi,theta_cap=0,psi_cap=psi)
    double leeway = atan2(v_old , u_old);
    double Vb = sqrt(u_old*u_old + v_old*v_old);

    X0 = X004();
    F_app = FM_add();
    F_rudder = FM_rudder();
    F_sail = FM_sail();

    double force_X,force_Y;
    double moment_K,moment_N;

    //Vb=sqrt(U^2+V^2);
    force_X  = X0+F_app[0]+par.Xvdpsi*(0.5*par.rouwater*pow(par.lwl,2)*par.draftwithkeel)*v_old*dpsi+F_rudder[0]+F_sail[0];
    force_Y  = F_app[1]+par.Ydfi*(0.5*par.rouwater*pow(par.lwl,2)*par.draftwithkeel*Vb)*dphi+par.Ydpsi*(0.5*par.rouwater*pow(par.lwl,2)*par.draftwithkeel*Vb)*dpsi+F_rudder[1]+F_sail[1];
    moment_K = F_app[2]+par.Kdfi*(0.5*par.rouwater*pow(par.lwl,2)*pow(par.draftwithkeel,2)*Vb)*dphi+F_rudder[2]+F_sail[2]-par.m*par.g*par.GMt*sin(phi_old);
    moment_N = F_app[3]+par.Ndpsi*(0.5*par.rouwater*pow(par.lwl,3)*par.draftwithkeel*Vb)*dpsi+F_rudder[3]+F_sail[3];
    //KH(here means the F_app)
    du = (force_X+(par.m+par.my*pow(cos(phi_old),2)+par.mz*pow(sin(phi_old),2))*v_old*dpsi)/(par.m+par.mx);
    dv = (force_Y-2*(par.mz-par.my)*sin(phi_old)*cos(phi_old)*v_old*dphi-(par.m+par.mx)*u_old*dpsi)/(par.m+par.my*pow(cos(phi_old),2)+par.mz*pow(sin(phi_old),2));
    ddphi=(moment_K+((par.Iyy+par.Jyy)-(par.Izz+par.Jzz))*sin(phi_old)*cos(phi_old)*pow(dpsi,2))/(par.Ixx+par.Jxx);
    //ddfi=(moment_K)/(par.Ixx+par.Jxx);
    ddpsi=(moment_N-2*((par.Iyy+par.Jyy)-(par.Izz+par.Jzz))*sin(phi_old)*cos(phi_old)*dpsi*dphi)/((par.Iyy+par.Jyy)*pow(sin(phi_old),2)+(par.Izz+par.Jzz)*pow(cos(phi_old),2));
}


double* CSimulationVer2::transformation() {
    MatrixXd R_b2n(3,3),R_hb2b(3,3);
    R_b2n << cos(psi),-sin(psi)*cos(phi),sin(psi)*sin(phi),
            sin(psi),cos(psi)*cos(phi),-cos(psi)*sin(phi),
            0,sin(phi),cos(phi);
    R_hb2b << 1,0,0,
            0,cos(phi),sin(phi),
            0,-sin(phi),cos(phi);
    MatrixXd vb0(3,1),va0(3,1);
    va0 << u,v,0;
    vb0 = R_b2n * R_hb2b * va0;

    double *uv0;
    uv0 = new double[2];
    uv0[0] = vb0(0,0);
    uv0[1] = vb0(1,0);

    return uv0;
}

double CSimulationVer2::limitsPi(double tmp) {
    while (fabs(tmp)>PI)
    {
        if(tmp>0)
            tmp = tmp-2*PI;
        else
            tmp = tmp+2*PI;
    }
    return tmp;
}

void CSimulationVer2::Sailboat_Calc(double d_t) {

    double delta_t = d_t;

    FourD0F();

    u = u_old + du * delta_t;
    v = v_old + dv * delta_t;
    dphi = dphi + ddphi * delta_t;
    dpsi = dpsi + ddpsi * delta_t;

    phi = phi_old + dphi * delta_t;
    psi = psi_old + dpsi * delta_t;

    psi = limitsPi(psi);

    u_old = u;   //m/s
    v_old = v;   //m/s
    phi_old = phi; //rad
    psi_old = psi; //rad

    east_old = east;  //m
    north_old = north; //m


    AWA_old = AWA;
    AWS_old = AWS;
}

void CSimulationVer2::SetOrigin(double u0, double v0, double phi0, double psi0, double e0, double n0) {
    u_old = u0;
    v_old = v0;
    phi_old = phi0;
    psi_old = psi0;
    east_old = e0;
    north_old =n0;
}
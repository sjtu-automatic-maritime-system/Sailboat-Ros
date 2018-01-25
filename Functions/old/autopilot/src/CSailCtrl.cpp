//
// Created by hywel on 17-5-5.
//

#include "autopilot/CSailCtrl.h"

CSailCtrl::CSailCtrl() {
    awaData = new double[19];
    sailAngleData1 = new double[19];
    sailAngleData2 = new double[19];
    //攻角 10 10 10 10 10 38 40 44 50 50 54 58 64 64 72 78 82 90
    double awaTmp[19]   = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180};
    double sailTmp1[19] = {0, 0,10,20,30,40,22,30,36,40, 50, 56, 62,66,76,78,82,88,90};
    double sailTmp2[19] = {0, 0, 5,10,15,20,22,30,36,40, 50, 55, 60,63,72,73,76,81,82};

    for (int i = 0; i < 19 ; ++i) {
        awaData[i] = awaTmp[i]/180*pi;
        sailAngleData1[i] = sailTmp1[i]/180*pi;
        sailAngleData2[i] = sailTmp2[i]/180*pi;
    }

    SIP1 = new CCubicSplineInterpolation(awaData,sailAngleData1,19);
    SIP2 = new CCubicSplineInterpolation(awaData,sailAngleData2,19);
}

CSailCtrl::~CSailCtrl() {
    delete [] awaData;
    delete [] sailAngleData1;
    delete [] sailAngleData2;
    delete SIP1;
    delete SIP2;
}

double CSailCtrl::GetBestSailAngle1(double awa) {
    double sail;
    if(awa >= 0)
    {
        sail = SIP1->GetYByX(awa);
        sail = -fabs(sail);
    }
    else
    {
        sail = SIP1->GetYByX(-awa);
        sail = fabs(sail);
    }
    return sail;
}

double CSailCtrl::GetBestSailAngle2(double awa) {
    double sail;
    if(awa >= 0)
    {
        sail = SIP2->GetYByX(awa);
        sail = -fabs(sail);
    }
    else
    {
        sail = SIP2->GetYByX(-awa);
        sail = fabs(sail);
    }
    return sail;
}
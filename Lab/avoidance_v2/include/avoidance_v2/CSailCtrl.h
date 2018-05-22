//
// Created by hywel on 17-5-5.
//

#ifndef SAILBOAT_CSAILCTRL_H
#define SAILBOAT_CSAILCTRL_H

#endif //SAILBOAT_CSAILCTRL_H

#include "math_tool_lib/CCubicSplineInterpolation.h"
#include <cmath>

class CSailCtrl{
public:

    CSailCtrl();
    ~CSailCtrl();

    //原始最佳帆船数据
    double GetBestSailAngle1(double awa);
    //处理后
    double GetBestSailAngle2(double awa);

private:
    double AWA;
    double SailAngle;

    double * awaData;
    double * sailAngleData1;
    double * sailAngleData2;
    CCubicSplineInterpolation *SIP1;
    CCubicSplineInterpolation *SIP2;
};
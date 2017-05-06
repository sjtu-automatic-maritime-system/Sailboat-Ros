//
// Created by hywel on 17-5-5.
//

#ifndef SAILBOAT_CSAILCTRL_H
#define SAILBOAT_CSAILCTRL_H

#endif //SAILBOAT_CSAILCTRL_H
#define pi 3.1415926

class CSailCtrl{
public:
    CSailCtrl();
    ~CSailCtrl();

private:
    double AWA;
    double SailAngle;

    double * awaData;
    double * sailAngleData;
};
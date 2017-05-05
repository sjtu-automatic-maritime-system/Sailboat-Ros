//
// Created by hywel on 17-5-5.
//

#include "CSailCtrl.h"

CSailCtrl::CSailCtrl() {
    awaData = new double[19];
    sailAngleData = new double[19];

    //攻角 10 10 10 10 10 38 40 44 50 50 54 58 64 64 72 78 82 90
    double awaTmp[19] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180};
    double sailTmp[19] = {0,0,10,20,30,40,22,30,36,40,50,56,62,66,76,78,82,88,90};

    for (int i = 0; i < 19 ; ++i) {
        awaData[i] = awaTmp[i]/180*pi;
        sailAngleData[i] = sailTmp[i]/180*pi;
    }

}

CSailCtrl::~CSailCtrl() {
    delete [] awaData;
    delete [] sailAngleData;
}
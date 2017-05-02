//
// Created by hywel on 17-4-28.
//

// CCubicSplineInterpolation.h: interface for the CCubicSplineInterpolation class.
//
/*/////////////////////////////////////////////////////////////////////
*
*class CCubicSplineInterpolation  三次样插值计算类
*使用接口
* CCubicSplineInterpolation(double *ctrlPtX,double *ctrlPtY,int nCtrlPtCount);  输入控制点，构造函数
* bool GetInterpolationPts(int outPtCount,double* outPtX,double* outPtY);       获得插值后的点数组
* bool GetYByX(const double &dbInX, double &dbOutY);                            获得单个X值所对应的Y值，即反算
*
*//////////////////////////////////////////////////////////////////////
#ifndef SAILBOAT_CCUBICSPLINEINTERPOLATION_H
#define SAILBOAT_CCUBICSPLINEINTERPOLATION_H

#include <iostream>
#include <string.h>
#include <math.h>

using namespace std;

class CCubicSplineInterpolation
{
public:

    //////////////////////////////////////////////////////////////////////
    // Construction/Destruction
    //
    //ctrlPtX 控制点X数组
    //ctrlPtY 控制点Y数组
    //nCtrlPtCount 控制点数目，控制点要大于等于3.
    //
    //////////////////////////////////////////////////////////////////////
    CCubicSplineInterpolation(double *ctrlPtX,double *ctrlPtY,int nCtrlPtCount);
    virtual ~CCubicSplineInterpolation();
public:

    //////////////////////////////////////////////////////////////////////////
    //outPtCount 想要输出的插值点数目,输出的点数组要大于1
    //outPtX     已经分配好内存的X值的数组。
    //outPtY     已经分配好内存的Y值的数组。
    //
    //调用此函数，获得插值点数组
    //
    //计算成功返回true，计算失败返回false
    //////////////////////////////////////////////////////////////////////////
    bool GetInterpolationPts(int outPtCount,double* outPtX,double* outPtY);
    //////////////////////////////////////////////////////////////////////////
    //根据X值 计算Y值
    //dbInX   x自变量值，输入
    //dbOutY  计算得到的Y值 输出
    //////////////////////////////////////////////////////////////////////////
    double GetYByX(double dbInX);
protected:
    void ReleaseMem();
    void InitParam();
    bool InterPolation();
    bool Spline();
protected:
    bool m_bCreate; //类是否创建成功，即控制点是否有效
    int N;   //输入控制点数量
    int M;   //输出的插入点数量
    typedef double* pDouble;
    pDouble X,Y; //输入的控制点数组
    pDouble Z,F; //输出的控制点数组
    pDouble H,A,B,C,D; //间距，缓存运算中间结果。
};

#endif //SAILBOAT_CCUBICSPLINEINTERPOLATION_H

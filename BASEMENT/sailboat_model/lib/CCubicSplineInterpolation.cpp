//
// Created by hywel on 17-4-28.
//

// CubicSplineInterpolation.cpp: implementation of the CCubicSplineInterpolation class.
//
//////////////////////////////////////////////////////////////////////


#include "CCubicSplineInterpolation.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//
//ctrlPtX 控制点X数组
//ctrlPtY 控制点Y数组
//nCtrlPtCount 控制点数目，控制点要大于等于3.
//
//////////////////////////////////////////////////////////////////////

CCubicSplineInterpolation::CCubicSplineInterpolation(double *ctrlPtX,double *ctrlPtY,int nCtrlPtCount)
{
    InitParam();

    if (NULL == ctrlPtX || NULL == ctrlPtY || nCtrlPtCount < 3 )
    {
        m_bCreate = false;
    }
    else
    {
        N = nCtrlPtCount - 1;

        int nDataCount = N + 1;
        X = new double[nDataCount];
        Y = new double[nDataCount];

        A = new double[nDataCount];
        B = new double[nDataCount];
        C = new double[nDataCount];
        D = new double[nDataCount];
        H = new double[nDataCount];

        memcpy(X,ctrlPtX,nDataCount*sizeof(double));
        memcpy(Y,ctrlPtY,nDataCount*sizeof(double));

        m_bCreate = Spline();
    }
}

//////////////////////////////////////////////////////////////////////////
//outPtCount 想要输出的插值点数目,输出的点数组要大于1
//outPtX     已经分配好内存的X值的数组。
//outPtY     已经分配好内存的Y值的数组。
//
//调用此函数，获得插值点数组
//
//计算成功返回true，计算失败返回false
//////////////////////////////////////////////////////////////////////////
bool CCubicSplineInterpolation::GetInterpolationPts(int outPtCount, double *outPtX, double *outPtY)
{
    if (!m_bCreate)
    {
        return m_bCreate;
    }

    M = outPtCount - 1;

    if (M == 0)
    {
        return false;
    }

    Z = outPtX;
    F = outPtY;



    return InterPolation();


}

CCubicSplineInterpolation::~CCubicSplineInterpolation()
{
    ReleaseMem();
}

void CCubicSplineInterpolation::InitParam()
{
    X = Y = Z = F = A = B = C = D = H = NULL;

    N = 0;
    M = 0;
}

void CCubicSplineInterpolation::ReleaseMem()
{
    delete [] X;
    delete [] Y;
    //  delete [] Z;
    //  delete [] F;
    delete [] A;
    delete [] B;
    delete [] C;
    delete [] D;
    delete [] H;

    InitParam();
}


bool CCubicSplineInterpolation::Spline()
{
    int i,P,L;

    for (i=1;i<=N;i++)
    {
        H[i-1]=X[i]-X[i-1];
    }

    L=N-1;
    for(i=1;i<=L;i++)
    {
        A[i]=H[i-1]/(H[i-1]+H[i]);
        B[i]=3*((1-A[i])*(Y[i]-Y[i-1])/H[i-1]+A[i]*(Y[i+1]-Y[i])/H[i]);
    }
    A[0]=1;
    A[N]=0;
    B[0]=3*(Y[1]-Y[0])/H[0];
    B[N]=3*(Y[N]-Y[N-1])/H[N-1];

    for(i=0;i<=N;i++)
    {
        D[i]=2;
    }

    for(i=0;i<=N;i++)
    {
        C[i]=1-A[i];
    }

    P=N;
    for(i=1;i<=P;i++)
    {

        if (  fabs(D[i]) <= 0.000001 )
        {
            return false;
            //    MessageBox(0,"无解","提示,MB_OK);
            //break;
        }
        A[i-1]=A[i-1]/D[i-1];
        B[i-1]=B[i-1]/D[i-1];
        D[i]=A[i-1]*(-C[i])+D[i];
        B[i]=-C[i]*B[i-1]+B[i];
    }
    B[P]=B[P]/D[P];
    for(i=1;i<=P;i++)
    {
        B[P-i]=B[P-i]-A[P-i]*B[P-i+1];
    }

    return true;
}

bool CCubicSplineInterpolation::InterPolation()
{

    double dbStep = (X[N] - X[0])/(M);

    for (int i = 0;i <= M ;++i)
    {
        Z[i] = X[0] + dbStep*i;
    }

    for(int i=1;i<=M;i++)
    {
        F[i] = GetYByX(Z[i]);
    }

    F[0] = Y[0];

    return true;
}


double CCubicSplineInterpolation::GetYByX(double dbInX)
{

    double dbOutY;
    if (!m_bCreate)
    {
        cout<<"error"<<endl;
        dbOutY = 0;
        return dbOutY;

    }

    double E,E1,K,K1,H1;
    int j ;
    if(dbInX<X[0])
    {
        j = 0;

    }
    else if (dbInX > X[N])
    {
        j = N-1;
    }
    else
    {
        for (j=1;j<=N;j++)
        {
            if(dbInX<=X[j])
            {
                j=j-1;

                break;
            }
        }

    }

    //////////////////////////////////////////////////////////////////////////
    E=X[j+1]-dbInX;
    E1=E*E;
    K=dbInX-X[j];
    K1=K*K;
    H1=H[j]*H[j];

    dbOutY=(3*E1-2*E1*E/H[j])*Y[j]+(3*K1-2*K1*K/H[j])*Y[j+1];
    dbOutY=dbOutY+(H[j]*E1-E1*E)*B[j]-(H[j]*K1-K1*K)*B[j+1];
    dbOutY=dbOutY/H1;

    return dbOutY;
}
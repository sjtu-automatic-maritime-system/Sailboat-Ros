//
// Created by hywel on 17-5-1.
//
#include "sailboat_simulation_lib/CSimulationVer1.h"


int main(){
    double t;
    double d_t=0.02;
    cin>>t;
    CSimulationVer1 SME;


    SME.Sailboat_Test(t,d_t);

    cout<<fabs(0.1)<<endl;
    cout<<abs(0.1)<<endl;

    return 0;
}
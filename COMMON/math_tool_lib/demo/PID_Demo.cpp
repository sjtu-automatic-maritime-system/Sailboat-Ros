//
// Created by sjtu on 17-4-23.
//

#include <iostream>
#include "math_tool_lib/CPID.h"
#include <unistd.h>

using namespace std;
int main()
{
    double out;
    CPID cpid(2,1,1,0.01,10,-10);
    cpid.Set_Ref(10);
    cpid.Set_Fdb(0);
    while(1){
        out = cpid.PID_Calc();
        cout<<out<<endl;
        sleep(1);
    }
    return 0;
}
//
// Created by hywel on 17-5-6.
//

#include "autopilot_lib/CSailCtrl.h"
#include <iostream>

using namespace std;
int main(){
    CSailCtrl testSail;
    for (int i = -36; i < 37; ++i) {
        double awaangle;
        awaangle = 1.0*i*5/180*pi;
        double sailangle1,sailangle2;
        sailangle1 = testSail.GetBestSailAngle1(awaangle);
        sailangle2 = testSail.GetBestSailAngle2(awaangle);
        cout<<awaangle<<" "<<sailangle1<<" "<<sailangle2<<endl;
    }
}

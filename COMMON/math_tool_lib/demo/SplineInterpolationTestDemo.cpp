//
// Created by hywel on 17-4-28.
//

#include "math_tool_lib/CCubicSplineInterpolation.h"

int main()
{
    double xdata[37];
    double yldata[37]= {0, 0.3212, 0.683, 0.8553, 0.9999, 1.095, 0.652, 0.6738, 0.7474,
                        0.8103, 0.8848, 0.9566, 1.024, 1.085, 1.14, 1.186, 1.222, 1.248, 1.018,
                        0.7698, 0.7468, 0.7174, 0.6837, 0.6453, 0.6056, 0.5646, 0.5223, 0.4784,
                        0.4329, 0.3858, 0.3368, 0.286, 0.2331, 0.1782, 0.1211, 0.0617, 0};
    double yddata[37] = {0.0072, 0.009, 0.012, 0.0161, 0.0224, 0.0324, 0.1509, 0.1952, 0.2271,
                         0.2624, 0.2917, 0.3198, 0.3469, 0.3721, 0.3962, 0.418, 0.4338, 0.4466,
                         0.5092, 0.576, 0.5956, 0.6071, 0.6147, 0.6325, 0.663, 0.7062, 0.7621,
                         0.8305, 0.9114, 1.005, 1.11, 1.229, 1.359, 1.501, 1.655, 1.822, 2.0};
    for (int i = 0; i < 37 ; i++) {
        xdata[i]=i*2.5;
    }


    int countx = sizeof(xdata) / sizeof(xdata[0]);
    int county = sizeof(yldata) / sizeof(yldata[0]);

    cout<<countx<<endl;
    cout<<county<<endl;

    CCubicSplineInterpolation SIP(xdata,yldata,county);

    double p1, p2, p3;
    p1 =4;
    p2 =5;
    p3 =6;


    double outY1,outY2,outY3;
    outY1 = SIP.GetYByX(p1);
    outY2 = SIP.GetYByX(p2);
    outY3 = SIP.GetYByX(p3);

    cout<<outY1<<endl;
    cout<<outY2<<endl;
    cout<<outY3<<endl;


}

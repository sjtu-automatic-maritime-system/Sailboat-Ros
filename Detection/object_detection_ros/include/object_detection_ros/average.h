#ifndef AVERAGE_H
#define AVERAGE_H
#include "ros/ros.h"
#include "cmath"
#include <vector>

struct pointData2{
    double x;
    double y;
};

// struct objectPoseArray{
    
// };

class Average{
public:
    Average();
    ~Average();
    void addObeject(double x, double y);
    int inputDetectBall(double x, double y);
    bool isDetectBallInArray(double x, double y, int &index);

    void run(double x, double y,double yaw, double &bestX, double &bestY);
    void publish();
private:
    std::vector< pointData2 > objectPoseArray;
    double ball_num;
};

#endif
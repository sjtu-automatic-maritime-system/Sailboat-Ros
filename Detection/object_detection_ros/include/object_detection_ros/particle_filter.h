#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include "ros/ros.h"
#include "cmath"
#include <vector>
//using namespace std;

struct pointData{
    double x;
    double y;
    double probability;
};

struct pointArray{
    std::vector< pointData > pointDataArray;

    double bestX;
    double bestY;
    
};
class ParticleFilter{
public:
    ParticleFilter();
    ~ParticleFilter();
    
    void createParticles(double x, double y);
    int inputDetectBall(double x, double y);
    bool isDetectBallInArray(double x, double y, int &index);
    
    void updateWeight(double x, double y, int index);
    void estimate(int index);
    double neff(int index);
    double resampleFromIndex(int index);

    static bool SortByM1( const pointData &v1, const pointData &v2);

    void run(double x, double y, double &bestX, double &bestY);
    void publish();
private:
    
    std::vector< pointArray > ball_point_set;

    int ball_num;
    double ball_r;
    double sigma;

};

#endif

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
    double last_yaw;

    int detectNum;
    
};

struct objectPoseArray{
    std::vector< pointData > poseArray;
};

class ParticleFilter{
public:
    ParticleFilter();
    ~ParticleFilter();
    
    void createParticles(double x, double y);
    int inputDetectBall(double x, double y);
    bool isDetectBallInArray(double x, double y, int &index);
    
    void updateWeight(double x, double y,double yaw, int index);
    void estimate(int index);
    double neff(int index);
    double resampleFromIndex(double x, double y,double yaw, int index);

    static bool SortByM1( const pointData &v1, const pointData &v2);

    void run(double x, double y,double yaw, double &bestX, double &bestY,double &last_yaw);
    void publish(objectPoseArray &object_pose_array);

    void deleteObjet();
    double calDistance(double x1,double y1,double x2,double y2);
private:
    
    std::vector< pointArray > ball_point_set;
    //std::vector< pointData > object_pose_array;
    int ball_num;
    double ball_r;
    double sigma1;
    double sigma2;

    int setReFrom;

};

#endif

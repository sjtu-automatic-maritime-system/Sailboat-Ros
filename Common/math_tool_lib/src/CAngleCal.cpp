#include <math.h>
//#include <eigen3/Eigen/Dense>
#include <Eigen/Core>

//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace std;

//roll pitch yaw
void roll_pitch_yaw_to_R(Vector3d E,Matrix3d &R){
    double sx = sin(E(0));
    double cx = cos(E(0));
    double sy = sin(E(1));
    double cy = cos(E(1));
    double sz = sin(E(2));
    double cz = cos(E(2));
    R(0,0) = cy*cz;
    R(0,1) = cz*sx*sy-cx*sz;
    R(0,2) = sx*sz+cx*cz*sy;
    R(1,0) = cy*sz;
    R(1,1) = cx*cz+sx*sy*sz;
    R(1,2) = cx*sy*sz−cz*sz;
    R(2,0) = −sy;
    R(2,1) = cy*sx;
    R(2,2) = cx*cy;
    cout<<"R=\n"<<R<<endl;
}


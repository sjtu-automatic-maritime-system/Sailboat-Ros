//
// Created by jianyun on 17-10-18.
//

#include <iostream>
#include <fstream>
#include "Astar.h"
#include <math.h>
#include "sensor_fusion_lib/measurement_package.h"
#include "sensor_fusion_lib/tracking.h"

using namespace std;

int main() {
    //初始化地图，用二维矩阵代表地图，1表示障碍物，0表示可通
//    vector<vector<int> > maze = {
//            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//            {1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1},
//            {1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
//            {1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},
//            {1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
//            {1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//            {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
//            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
//    };
    vector<vector<int> > maze = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0},
            {0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0},
            {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
//    double windAngle = -M_PI_2; // east wind
//    double windAngle = 0;       // south wind
//    double windAngle = M_PI_2;  // west wind
//    double windAngle = M_PI;    // north wind
//    double windAngle = -M_PI_4; // east-south wind
//    double windAngle = M_PI_4; // west-south wind
//    double windAngle = -M_PI_4*3; // east-north wind
//    double windAngle = M_PI_4 * 3; // west-north wind

    std::vector<double> windAngleVec = {-M_PI_2, 0, M_PI_2, M_PI, -M_PI_4, M_PI_4, -M_PI_4 * 3, M_PI_4 * 3};
    std::vector<string> windNameVec = {"east_wind", "south_wind", "west_wind", "north_wind", "east_south_wind",
                                       "west_south_wind", "east_north_wind", "west_north_wind"};

//    double heading = -M_PI_2;
    std::vector<double> headingVec = {-M_PI_2, 0, M_PI_2, M_PI, -M_PI_4, M_PI_4, -M_PI_4 * 3, M_PI_4 * 3};
    std::vector<string> headingNameVec = {"west", "north", "east", "south", "west_north",
                                          "east_north", "west_south", "east_south"};
    Astar astar;

    Tracking tracking;
    Eigen::MatrixXd KF_R(2, 2); //measurement covariance
    KF_R << 1, 0,
            0, 1;
    float noise_ax = 0.001;
    float noise_ay = 0.001;
    MeasurementPackage measurement;
    measurement.sensor_type_ = MeasurementPackage::LASER;

    ofstream pathfile;
    pathfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/path.txt");
    for (int j = 0; j < headingVec.size(); j++) {
//    for (int j = 0; j < 1; j++) {
        double heading = headingVec[j];
        double init_vx = 2.0 * cos(heading);
        double init_vy = 2.0 * sin(heading);
        tracking.InitTracking(KF_R, noise_ax, noise_ay, init_vx, init_vy);
        for (int i = 0; i < windAngleVec.size(); i++) {
//        for (int i = 0; i < 1; i++) {
            double windAngle = windAngleVec[i];
            astar.InitAstar(maze, windAngle, heading);
            //设置起始和结束点
            Point start(1, 1);
            //Point end(6, 10);
            Point end(7, 11);
            //A*算法找寻路径
            list<Point *> path = astar.GetPath(start, end, false);

            for (auto p:path) {

            }

            //打印
            pathfile << "heading" << "," << headingNameVec[j] << "," << heading << endl;
            pathfile << "wind" << "," << windNameVec[i] << "," << windAngle << endl;
//            pathfile << maze.size() << "," << maze[0].size() << endl;
            cout << "path size: " << path.size() << endl;
            float t = 0.0;
            for (auto &p:path) {
                cout << '(' << p->x << ',' << p->y << ')' << endl;

                t += 0.1;
                measurement.timestamp_ = t;
                measurement.raw_measurements_ = Eigen::VectorXd(2);
                measurement.raw_measurements_ << p->x, p->y;
                tracking.ProcessMeasurement(measurement);
                float x_f = tracking.kf_.x_[0];
                float y_f = tracking.kf_.x_[1];


                pathfile << p->x << ',' << p->y << ',' << x_f << ',' << y_f << endl;

            }
            pathfile << "####" << endl;
        }
    }
    pathfile.close();

    ofstream mapfile;
    mapfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/map.txt");
    for (auto &row:maze) {
        for (auto p:row)
            mapfile << p << ' ';
        mapfile << endl;
    }
    mapfile.close();
    return 0;
}
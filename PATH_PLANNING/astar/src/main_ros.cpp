//
// Created by jianyun on 17-10-18.
//

#include <iostream>
#include <fstream>
#include "Astar.h"
#include <math.h>

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
//    double windAngle = -M_PI_4;
    double windAngle = -0;
    double heading = -M_PI_2;
    Astar astar;
    astar.InitAstar(maze, windAngle, heading);

    //设置起始和结束点
    Point start(1, 1);
    start.heading = heading;
    std::cout<< "start: " << start.heading << endl;
//    Point end(6, 10);
    Point end(7, 11);
    //A*算法找寻路径
    list<Point *> path = astar.GetPath(start, end, false);
    //打印
    ofstream pathfile;
    pathfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/path.txt");
    pathfile << maze.size() << ',' << maze[0].size() << endl;
    for (auto &p:path) {
        cout << '(' << p->x << ',' << p->y << ')' << endl;
        pathfile << p->x << ',' << p->y << endl;
    }
    pathfile.close();
    ofstream mapfile;
    mapfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/map.txt");
    for (auto &row:maze) {
        for (auto p:row)
            mapfile << p << ' ';
        mapfile << endl;
    }
    return 0;
}
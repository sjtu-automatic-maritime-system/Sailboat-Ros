//
// Created by jianyun on 17-10-18.
//
#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include "Astar.h"
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include "sailboat_message/WTST_msg.h"
#include "nav_msgs/Path.h"
#include "sailboat_message/GPS_msg.h"


#define __SAVE_FILE__


ros::Publisher pub_path;


double extend = 10;
double resolution = 2;
Astar astar;
Eigen::Vector2d obs_gps_ne(0, 0);


using namespace std;

Eigen::Vector2i ned2map(Eigen::Vector2d &ne, Eigen::Vector2d &origin,
                        int n_row, int n_col, double resolution) {
    int row = (int) (ne[0] - origin[0]) / resolution;
    row = n_row - 1 - row;
    row = min(max(0, row), n_row);
    int col = (int) (ne[1] - origin[1]) / resolution;
    col = min(max(0, col), n_col);
    Eigen::Vector2i map_coord(row, col);
    return map_coord;
}

Eigen::Vector2d map2ned(Eigen::Vector2i &map_coord, Eigen::Vector2d &origin_ne,
                        int n_row, int n_col, double resolution) {
    int row = n_row - 1 - map_coord[0];
    row = min(max(0, row), n_row);
    double north = origin_ne[0] + row * resolution;
    int col = map_coord[1];
    col = min(max(0, col), n_col);
    double east = origin_ne[1] + col * resolution;
    Eigen::Vector2d ne(north, east);
    return ne;
}

vector<vector<int> > mapGeneration(int n_row, int n_col, vector<Eigen::Vector2i> objs_map) {
    vector<vector<int> > map;
    for (int row = 0; row < n_row; row++) {
        vector<int> tmp;
        for (int col = 0; col < n_col; col++) {
            tmp.push_back(0);
        }
        map.push_back(tmp);
    }
    for (auto obj_map:objs_map) {
        map[obj_map[0]][obj_map[1]] = 1;
        map[max(obj_map[0] - 1, 0)][obj_map[1]] = 1;
        map[min(obj_map[0] + 1, n_row)][obj_map[1]] = 1;
        map[obj_map[0]][max(obj_map[1] - 1, 0)] = 1;
        map[obj_map[0]][min(obj_map[1] + 1, n_col)] = 1;
    }
    return map;
}

void obs_gps_cb(const sailboat_message::GPS_msgConstPtr &obs_gps_in) {
    obs_gps_ne[0] = obs_gps_in->posx;
    obs_gps_ne[1] = obs_gps_in->posy;
}

void wtst_cb(const sailboat_message::WTST_msgConstPtr &wtst_in) {
    double heading = (wtst_in->Yaw) / 57.3;
//    double wind = wtst_in->TrueWindAngle;
    double wind = (wtst_in->WindAngle) / 57.3 + heading;
//    cout << heading << ", " << wind << endl;
//    wind = -M_PI_4 * 1.4;

    Eigen::Vector2d start_ne(wtst_in->PosX, wtst_in->PosY);
    Eigen::Vector2d end_ne(-50.0, 30.0);
//
    double x_small = min(start_ne[0], end_ne[0]);
    double x_big = max(start_ne[0], end_ne[0]);
    double y_small = min(start_ne[1], end_ne[1]);
    double y_big = max(start_ne[1], end_ne[1]);
    int n_row = (int) (x_big - x_small + 2 * extend) / resolution + 1;
    int n_col = (int) (y_big - y_small + 2 * extend) / resolution + 1;
    Eigen::Vector2d origin_ne(x_small - extend, y_small - extend);
    Eigen::Vector2i start_map = ned2map(start_ne, origin_ne, n_row, n_col, resolution);
//    cout << "start_map: " << start_map[0] << ", " << start_map[1] << endl;
    Eigen::Vector2i end_map = ned2map(end_ne, origin_ne, n_row, n_col, resolution);
//    cout << "end_map: " << end_map[0] << ", " << end_map[1] << endl;
//

    vector<Eigen::Vector2i> objs_map;
//    Eigen::Vector2d obj_ne_1(-30, 5);
//    objs_map.push_back(ned2map(obj_ne_1, origin_ne, n_row, n_col, resolution));
//    Eigen::Vector2d obj_ne_2(-40, 10);
//    objs_map.push_back(ned2map(obj_ne_2, origin_ne, n_row, n_col, resolution));
//    Eigen::Vector2d obj_ne_3(-25, -4);
//    objs_map.push_back(ned2map(obj_ne_3, origin_ne, n_row, n_col, resolution));
//    Eigen::Vector2d obj_ne_4(-50, 25);
//    objs_map.push_back(ned2map(obj_ne_4, origin_ne, n_row, n_col, resolution));
//    Eigen::Vector2d obj_ne_5(-35, 5);
//    objs_map.push_back(ned2map(obj_ne_5, origin_ne, n_row, n_col, resolution));
//    Eigen::Vector2d obj_ne_6(-30, 0);
//    objs_map.push_back(ned2map(obj_ne_6, origin_ne, n_row, n_col, resolution));

    // obstacle from another gps
    cout << "obs_gps: " << obs_gps_ne[0] << ", " << obs_gps_ne[1] << endl;
    objs_map.push_back(ned2map(obs_gps_ne, origin_ne, n_row, n_col, resolution));

    vector<vector<int> > maze = mapGeneration(n_row, n_col, objs_map);

    //设置起始和结束点
    Point start(start_map[0], start_map[1]);
    //Point end(6, 10);
    Point end(end_map[0], end_map[1]);
    astar.InitAstar(maze, wind, heading);

    //A*算法找寻路径
    list<Point *> path = astar.GetPath(start, end, false);
    cout << "###############" << endl;
    nav_msgs::Path traj;
    for (auto &p:path) {
        cout << "(" << p->x << ',' << p->y << ')' << endl;

        Eigen::Vector2i pt_map(p->x, p->y);
        Eigen::Vector2d pt_ne = map2ned(pt_map, origin_ne, n_row, n_col, resolution);

        geometry_msgs::PoseStamped pose_to_path;
//        pose_to_path.header.stamp = wtst_in->header.stamp;
        pose_to_path.pose.position.x = pt_ne[1];
        pose_to_path.pose.position.y = pt_ne[0];
        traj.poses.push_back(pose_to_path);
    }
    traj.header = wtst_in->header;
    traj.header.frame_id = "world";
    pub_path.publish(traj);


#ifdef __SAVE_FILE__
    ofstream pathfile;
    cout << "saving file" << endl;
    pathfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/path_new.txt");
    pathfile << "heading" << "," << heading << endl;
    pathfile << "wind" << "," << wind << endl;
    for (auto &p:path) {
        pathfile << p->x << ',' << p->y << endl;
    }
    pathfile << "####" << endl;

    ofstream mapfile;
    mapfile.open("/home/jianyun/catkin_ws/src/Sailboat-Ros/PATH_PLANNING/python/map_new.txt");
    for (auto &row:maze) {
        for (auto p:row)
            mapfile << p << ' ';
        mapfile << endl;
    }
    mapfile.close();
#endif

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "path_planning_astar");
    ros::NodeHandle nh;

    ros::Subscriber wtst_sub = nh.subscribe("/wtst", 2, &wtst_cb);
    ros::Subscriber obs_gps_sub = nh.subscribe("/gps_2", 2, &obs_gps_cb);

    pub_path = nh.advertise<nav_msgs::Path>("/planned_path", 2);

    ros::spin();
    return 0;
}
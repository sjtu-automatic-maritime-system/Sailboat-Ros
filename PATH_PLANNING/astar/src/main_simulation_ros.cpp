//
// Created by jianyun on 17-10-18.
//
#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include "Astar.h"
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include "sailboat_message/WTST_msg.h"
#include "nav_msgs/Path.h"
#include "sailboat_message/GPS_msg.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Wind_Simulation_msg.h"

#include "sailboat_message/Point.h"
#include "sailboat_message/PointArray.h"

#include <dynamic_reconfigure/server.h>
#include "path_planning_astar/path_planning_Config.h"


//#define __SAVE_FILE__

using namespace std;


ros::Publisher pub_path;
ros::Publisher pub_obs;

sailboat_message::PointArray obs_coords_to_pub;


static double extend = 10;
static double resolution = 2;
static double loop_T = 2.0;

static double end_x = -60;
static double end_y = 20;

Astar astar;

std::vector<Eigen::Vector2d> obs_ne_vector;
std::vector<Eigen::Vector2d> target_points_ne;

std::list<double> wind_list;
double true_wind = 0;

int cnt = 0;
int tar_cnt = 0;


Eigen::Vector2i ned2map(Eigen::Vector2d &ne, Eigen::Vector2d &origin,
                        int n_row, int n_col, double resolution) {
    int row = (int) (ne[0] - origin[0]) / resolution;
    row = n_row - 1 - row;
    row = min(max(0, row), n_row-1);
    int col = (int) (ne[1] - origin[1]) / resolution;
    col = min(max(0, col), n_col-1);
    Eigen::Vector2i map_coord(row, col);
    return map_coord;
}

Eigen::Vector2d map2ned(Eigen::Vector2i &map_coord, Eigen::Vector2d &origin_ne,
                        int n_row, int n_col, double resolution) {
    int row = n_row - 1 - map_coord[0];
    row = min(max(0, row), n_row-1);
    double north = origin_ne[0] + row * resolution;
    int col = map_coord[1];
    col = min(max(0, col), n_col-1);
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
        map[min(obj_map[0] + 1, n_row-1)][obj_map[1]] = 1;
        map[obj_map[0]][max(obj_map[1] - 1, 0)] = 1;
        map[obj_map[0]][min(obj_map[1] + 1, n_col-1)] = 1;
    }
    cout << "in map genertion " << map[0][0] << endl;
    return map;
}

void cfg_cb(path_planning_astar::path_planning_Config &config, uint32_t level) {
    extend = config.map_extend;
    resolution = config.map_resolution;
    loop_T = config.loop_T;
    end_x = -config.end_x;
    end_y = config.end_y;
}


void wind_simulation_cb(const sailboat_message::Wind_Simulation_msgConstPtr &wind_sim_in) {
    true_wind = wind_sim_in->TWA;
}

void sensor_cb(const sailboat_message::Sensor_msgConstPtr &sensor_in) {
    cnt++;
//    if (cnt < 50) {
    if (cnt == (int) (loop_T / 0.1)) {
        cnt = 0;
        obs_coords_to_pub.points.clear();

        double heading = (sensor_in->Yaw);
//    double wind = wtst_in->TrueWindAngle;
        double wind = true_wind;
//        cout << "wind angle: " << ", " << wind << endl;


        Eigen::Vector2d start_ne(sensor_in->Posx, sensor_in->Posy);
        Eigen::Vector2d end_ne;
        if (tar_cnt < target_points_ne.size()) {
            end_ne = target_points_ne[tar_cnt];
            if (sqrt(pow(start_ne[0] - end_ne[0], 2) + pow(start_ne[1] - end_ne[1], 2)) < 2 &&
                tar_cnt + 1 < target_points_ne.size()) {
                tar_cnt++;
                cout << "changing target, cnt: " << tar_cnt << endl;
                end_ne[0] = target_points_ne[tar_cnt][0];
                end_ne[1] = target_points_ne[tar_cnt][1];
            }
        }
//        end_ne << end_x, end_y;
//        end_ne << -20, 20;
        cout << "start: " << start_ne[0] << ", " << start_ne[1] << endl;
        cout << "end  : " << end_ne[0] << ", " << end_ne[1] << endl;
//
        double x_small = min(start_ne[0], end_ne[0]);
        double x_big = max(start_ne[0], end_ne[0]);
        double y_small = min(start_ne[1], end_ne[1]);
        double y_big = max(start_ne[1], end_ne[1]);
        cout << "x_small: " << x_small << ", " << "y_small: " << y_small << endl;
        cout << "x_big: " << x_big << ", " << "y_big: " << y_big << endl;
        int n_row = (int) (x_big - x_small + 2 * extend) / resolution + 1;
        int n_col = (int) (y_big - y_small + 2 * extend) / resolution + 1;
        cout << "n_row: " << n_row << ", " << "n_col: " << n_col << endl;
        Eigen::Vector2d origin_ne(x_small - extend, y_small - extend);
        Eigen::Vector2i start_map = ned2map(start_ne, origin_ne, n_row, n_col, resolution);
        cout << "start_map: " << start_map[0] << ", " << start_map[1] << endl;
        Eigen::Vector2i end_map = ned2map(end_ne, origin_ne, n_row, n_col, resolution);
        cout << "end_map: " << end_map[0] << ", " << end_map[1] << endl;
//

        vector<Eigen::Vector2i> objs_map;
        for (auto p_ne:obs_ne_vector) {
            sailboat_message::Point p;
            p.x = p_ne[0];
            p.y = p_ne[1];
            obs_coords_to_pub.points.push_back(p);
//            cout << "obs_ne: " << p_ne[0] << ", " << p_ne[1] << endl;
            objs_map.push_back(ned2map(p_ne, origin_ne, n_row, n_col, resolution));
        }
        obs_coords_to_pub.header = sensor_in->header;
        obs_coords_to_pub.header.frame_id = "world";
        pub_obs.publish(obs_coords_to_pub);


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
//            cout << "(" << p->x << ',' << p->y << ')' << endl;

            Eigen::Vector2i pt_map(p->x, p->y);
            Eigen::Vector2d pt_ne = map2ned(pt_map, origin_ne, n_row, n_col, resolution);

            geometry_msgs::PoseStamped pose_to_path;
//        pose_to_path.header.stamp = wtst_in->header.stamp;
            pose_to_path.pose.position.x = pt_ne[1];
            pose_to_path.pose.position.y = pt_ne[0];
            traj.poses.push_back(pose_to_path);
        }
        traj.header = sensor_in->header;
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
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_planning_astar");
    ros::NodeHandle nh;

    // set obs points
    Eigen::Vector2d obj_ne_1(-30, 5);
    obs_ne_vector.push_back(obj_ne_1);
    Eigen::Vector2d obj_ne_2(-40, 10);
    obs_ne_vector.push_back(obj_ne_2);
    Eigen::Vector2d obj_ne_3(-25, -4);
    obs_ne_vector.push_back(obj_ne_3);
    Eigen::Vector2d obj_ne_4(-50, 25);
    obs_ne_vector.push_back(obj_ne_4);
    Eigen::Vector2d obj_ne_5(-35, 5);
    obs_ne_vector.push_back(obj_ne_5);
    Eigen::Vector2d obj_ne_6(-30, 0);
    obs_ne_vector.push_back(obj_ne_6);

    //set target points
    Eigen::Vector2d tar_1(-60, 20);
    target_points_ne.push_back(tar_1);
    Eigen::Vector2d tar_2(-50, -20);
    target_points_ne.push_back(tar_2);
    Eigen::Vector2d tar_3(-20, 0);
    target_points_ne.push_back(tar_3);
    Eigen::Vector2d tar_4(0, 0);
    target_points_ne.push_back(tar_4);
    sailboat_message::PointArray target_points_pub;
    for (auto p_ne:target_points_ne) {
        sailboat_message::Point p;
        p.x = p_ne[0];
        p.y = p_ne[1];
        target_points_pub.points.push_back(p);
    }


    for (int i = 0; i < 30; i++) {
        wind_list.push_back(0);
    }

    dynamic_reconfigure::Server<path_planning_astar::path_planning_Config> dserver;
    dynamic_reconfigure::Server<path_planning_astar::path_planning_Config>::CallbackType f;
    f = boost::bind(&cfg_cb, _1, _2);
    dserver.setCallback(f);

    ros::Subscriber sensor_sub = nh.subscribe("/sensor", 2, &sensor_cb);
    ros::Subscriber wind_sub = nh.subscribe("/wind", 2, &wind_simulation_cb);

    pub_path = nh.advertise<nav_msgs::Path>("/planned_path", 2);
    pub_obs = nh.advertise<sailboat_message::PointArray>("/obstacle_coords", 2);
    ros::Publisher pub_targets;
    pub_targets = nh.advertise<sailboat_message::PointArray>("/target_coords", 2);

    ros::Rate loo_rate(10);
    while (ros::ok()) {
        pub_targets.publish(target_points_pub);
        ros::spinOnce();
        loo_rate.sleep();
    }

    return 0;
}
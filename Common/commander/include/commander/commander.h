//
// Created by hywel on 11/9/17.
//

#ifndef PROJECT_COMMANDER_H
#define PROJECT_COMMANDER_H


#include <vector>
//math
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cfloat>

struct Commander_Status{
    float timestamp;
    int vehicle_control_state;
    //manual = 0;
    //auto_autopilot = 1;
    //auto_position = 2;
    //auto_pathfollow = 3;
    //auto_return = 4;
    //direct = 5;
    int path_planning_state;
    //manual_enable = 0;
    //astar_enable = 1;
    //int nav_state;//todo
    int sensor_state;
    int battery_state;
    int communication_state;
};

struct Commander_Cfg_Status{
    float timestamp;
    int vehicle_control_new_state;
    //manual = 0;
    //autopilot = 1;
    //position = 2;
    //pathfollow = 3;
    //return = 4;
    //direct = 5;
    int path_planning_new_state;
    //manual = 0;
    //astar = 1;
    //optimal_speed = 2;
    //direct = 3;
    bool is_auto; //最终控制权在遥控器
    bool is_reset_home;

    bool is_filter_control;
    bool is_tack_control;
    bool is_jibing_control;

    bool is_avoidance_plan;
    bool is_tack_plan;
};

struct Vehicle_Control_Status{
    float timestamp;
    int vehicle_control_state;
    int vehicle_control_new_state;
    bool is_transition;

    bool is_upwind;
    bool is_tack_time;
    bool is_jibing_time;
};

struct Path_Planning_Status{
    float timestamp;
    int path_planning_state;
    int path_planning_new_state;
    bool is_transition;

    int path_quality;
    //no path = 0;
    //setting path = 1;
    //avoidance path = 2;
    int point_number;
    int point_range;

    bool is_avoidance_time;
};

struct Navigation_Status{//todo
    float timestamp;
    int nav_state;
    int nav_new_state;
    // none navigation = 0

    //bool is_mission_doing;
};


struct Communication_Status{
    float timestamp;
    int communication_state;
    //arduino_lost and rc_lost = 0
    //arduino_lost and rc_get = 1
    //arduino_get and rc_lost = 2
    //arduino_get and rc_get = 3
    bool is_rc_get;
    bool is_arduino_get;
    bool is_wifi_get;

    float interrupt_time;
};

struct Sensor_Status{
    float timestamp;
    int sensor_state;
    //error = 0
    //warn = 1
    //no problem = 2
    bool is_camera_get;
    bool is_gps_get;
    bool is_ahrs_get;
    bool is_wind_get;
};

struct Camera_Status{
    float timestamp;
    int camera_state;
    //none = 0;
    //work = 1;
};


struct Battery_Status{
    float timestamp;
    int battery_state;
    //none = 0; # no battery low voltage warning active
    //good = 1;
    //low = 2; # warning of low voltage
    //emergency = 3; # immediate landing required
    float voltage_v_1;
    float voltage_v_2;
    float current_a_1;
    float current_a_2;
};


struct Vehicle_Control_Mode{
    float timestamp;
    bool flag_control_manual_enable;
    bool flag_control_auto_enable;
    bool flag_control_position_enable;
    bool flag_control_autoPilot_enable;
    bool flag_control_pathFollowing_enable;
    bool flag_control_return_enable;
    bool flag_control_direct_enable;

    bool flag_control_tack_enable;
    bool flag_control_jibing_enable;

};

struct Path_Planning_Mode{
    float timestamp;
    bool flag_planning_manual_enable;
    bool flag_planning_astar_enable;
    bool flag_planning_optimalSpeed_enable;
    bool flag_planning_direct_enable;

    bool flag_planning_avoidance_enable;
    bool flag_planning_tack_enable;
};

struct Home_Position{
    float timestamp;
    float posX;
    float posY;
    float longitude;
    float latitude;
};

struct Sensor_Msg{
    float timestamp;
    float ux;
    float vy;
    float gx;
    float gy;
    float gz;
    float posX;
    float posY;
    float roll;
    float pitch;
    float yaw;
    float awa;
    float aws;
};

struct Arduino_Msg{
    float timestamp;
    int readMark;
    int autoFlag;
    float motor;
    float rudder;
    float sail;
    float voltage1;
    float voltage2;
};

struct Path_Msg{
    float timestamp;
    vector <float> posX;
    vector <float> posY;
};

class Commander {
public:

    //main status
    Commander_Status commander_status;
    //cfg
    Commander_Cfg_Status vehicle_cfg_status;
    //mode_status
    Vehicle_Control_Status vehicle_control_status;
    Path_Planning_Status path_planning_status;
    Navigation_Status navigation_status;
    //sensor_status
    Sensor_Status sensor_status;
    Camera_Status camera_status;
    Battery_Status battery_status;
    Communication_Status communication_status;
    //publish control_mode
    Vehicle_Control_Mode vehicle_control_mode;
    Path_Planning_Mode path_planning_mode;
    //extra
    Home_Position home_position;
    //sensor
    Sensor_Msg sensor_msg;
    Arduino_Msg arduino_msg;
    Path_Msg path_msg;

    Commander();
    ~Commander();

    void init();

    void commander_thread_main();

    void vehicle_cfg_status_check();

    void sensor_status_check();

    void battery_status_check();

    void communication_status_check();

    void control_status_judge();

    void path_following_status_judge();

    void set_control_mode();

    void set_path_planning_mode();

    void print_status();

    static void commander_set_home_position();

    bool auto_required();

    //void set_navigation_mode();//todo

private:

};


#endif //PROJECT_COMMANDER_H

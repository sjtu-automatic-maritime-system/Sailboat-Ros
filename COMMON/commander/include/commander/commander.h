//
// Created by hywel on 11/9/17.
//

#ifndef PROJECT_COMMANDER_H
#define PROJECT_COMMANDER_H


class Commander {
public:

    struct Vehicle_Status{
        float timestamp;
        int vehicle_state;
        int vehicle_new_state;
        bool is_transition;

        int sensor_state;
        int battery_state;
        int communication_state;
        int path_planning_state;
        int nav_state;//todo

        bool is_upwind;
        bool is_tack_time;
        bool is_jibing_time;

    } vehicle_status;

    struct Vehicle_Cfg_Status{
        float timestamp;
        int vehicle_new_state;
        //manual = 0;
        //autopilot = 1;
        //position = 2;
        //pathfollow = 3;
        //direct = 4;
        bool is_auto_control; //最终控制权在遥控器
        bool is_filter_control;
        bool is_tack_control;
        bool is_jibing_control;
    } vehicle_main_cfg_status;

    struct Communication_Status{
        float timestamp;
        int communication_state;
        //wifi_lost and rc_lost = 0
        //wifi_lost and rc_get = 1
        //wifi_get and rc_lost = 2
        //wifi_get and rc_get = 3
        bool is_rc_get;
        bool is_wifi_get;
        bool is_arduino_get;
    } communication_status;

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

    struct Path_Planning_status{
        float timestamp;
        int path_planning_state;
        //no path = 0;
        //setting path = 1;
        //avoidance path = 2;

        int point_number;
        int point_range;
    } path_planning_status;

    struct Navigation_Status{//todo
        float timestamp;
        int nav_state;
        // none navigation = 0

        //bool is_mission_doing;
    } navigation_status;

    struct Battery_Status{
        float timestamp;
        int battery_state;
        //none = 0; # no battery low voltage warning active
        //low = 1; # warning of low voltage
        //critical = 2; # critical voltage, return / abort immediately
        //emergency = 3; # immediate landing required
        float voltage_v_1;
        float voltage_v_2;
        float current_a_1;
        float current_a_2;
    } battery_status;

    struct Vehicle_Control_Mode{
        float timestamp;
        bool flag_control_manual_enable;
        bool flag_control_auto_enable;
        bool flag_control_position_enable;
        bool flag_control_autopoilot_enable;
        bool flag_control_pathfollow_enable;
        bool flag_control_direct_enable;

        bool flag_control_tack_enable;
        bool flag_control_jibing_enable;

    } vehicle_control_mode;

    struct Home_Position{
        float posX;
        float posY;
        float longitude;
        float latitude;
    } home_position;

    struct Commander_Status{
        float timestamp;
        int main_state;
        //manual = 0;
        //auto_autopilot = 1;
        //auto_position = 2;
        //auto_pathfollow = 3;
        //direct = 4;
    } commander_status;


};


#endif //PROJECT_COMMANDER_H

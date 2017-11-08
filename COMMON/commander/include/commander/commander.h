//
// Created by hywel on 11/9/17.
//

#ifndef PROJECT_COMMANDER_H
#define PROJECT_COMMANDER_H


class Commander {
public:
    struct Commander_state{
        int main_state;
        //manual = 0;
        //auto_autopilot = 1;
        //auto_position = 2;
        //auto_pathfollow = 3;
        //direct = 4;
    };

    struct Vehicle_Status{
        int vehicle_state;
        //int nav_state;
        int sensor_state;

        int rc_input_mode;
        //bool is_transition_mode;
        bool is_mission_doing;
        bool is_rc_signal_get;

        bool is_gps_get;
        bool is_ahrs_get;
        bool is_wind_get;

        bool is_upwind;

    } vehicle_status;

    struct Vehicle_Control_Mode{
        bool flag_control_manual_enable;
        bool flag_control_auto_enable;
        bool flag_control_position_enable;
        bool flag_control_autopoilot_enable;
        bool flag_control_pathfollow_enable;
        bool flag_control_direct_enable;

        bool flag_control_tack_enable;
        bool flag_control_jibing_enable;

    } vehicle_control_mode;


    struct Battery_Status{
        float voltage_v_1;
        float voltage_v_2;
        float current_a_1;
        float current_a_2;

        int priority;
        //none = 0; # no battery low voltage warning active
        //low = 1; # warning of low voltage
        //critical = 2; # critical voltage, return / abort immediately
        //emergency = 3; # immediate landing required
    } battery_status;

    struct Home_Position{
        float posX;
        float posY;
        float longitude;
        float latitude;
    } home_position;

};


#endif //PROJECT_COMMANDER_H

//
// Created by hywel on 11/9/17.
//

#include <commander/commander.h>


Commander::Commander() {

    commander_status.timestamp = 0;
    commander_status.vehicle_control_state = 0;
    commander_status.path_planning_state = 0;
    commander_status.sensor_state = 0;
    commander_status.battery_state = 0;
    commander_status.communication_state = 0;
    //cfg
    vehicle_cfg_status.timestamp = 0;
    vehicle_cfg_status.vehicle_control_new_state = 0;
    vehicle_cfg_status.path_planning_new_state = 0;
    vehicle_cfg_status.is_auto = false;
    vehicle_cfg_status.is_reset_home = false;
    vehicle_cfg_status.is_filter_control = false;
    vehicle_cfg_status.is_tack_control = false;
    vehicle_cfg_status.is_jibing_control = false;
    vehicle_cfg_status.is_avoidance_plan = false;
    vehicle_cfg_status.is_tack_plan = false;
    //mode_status
    vehicle_control_status;

    path_planning_status;

    navigation_status;
    //sensor_status
    sensor_status;

    camera_status;

    battery_status;

    communication_status;
    //publish control_mode

    vehicle_control_mode;

    path_planning_mode;
    //extra

    home_position;
    //sensor

    sensor_msg;

    arduino_msg;

    path_msg;
}

Commander::~Commander() {

}
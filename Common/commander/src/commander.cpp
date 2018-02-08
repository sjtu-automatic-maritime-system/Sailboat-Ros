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
    vehicle_control_status.timestamp = 0;
    vehicle_control_status.vehicle_control_state = 0;
    vehicle_control_status.vehicle_control_new_state = 0;
    vehicle_control_status.is_transition = false;
    vehicle_control_status.is_upwind = false;
    vehicle_control_status.is_tack_time = false;
    vehicle_control_status.is_jibing_time = false;

    path_planning_status.timestamp = 0;
    path_planning_status.path_planning_state = 0;
    path_planning_status.path_planning_new_state = 0;
    path_planning_status.is_transition = false;
    path_planning_status.path_quality = 0;
    path_planning_status.point_number = 0;
    path_planning_status.point_range = 0;
    path_planning_status.is_avoidance_time = false;

    navigation_status.timestamp = 0;
    navigation_status.nav_state = 0;
    navigation_status.nav_new_state = 0;
    //sensor_status
    sensor_status.timestamp = 0;
    sensor_status.sensor_state = 0;
    sensor_status.is_camera_get = false;
    sensor_status.is_gps_get = false;
    sensor_status.is_ahrs_get = false;
    sensor_status.is_wind_get = false;

    camera_status.timestamp = 0;
    camera_status.camera_state = 0;

    battery_status.timestamp = 0;
    battery_status.battery_state = 0;
    battery_status.voltage_v_1 = 0;
    battery_status.voltage_v_2 = 0;
    battery_status.current_a_1 = 0;
    battery_status.current_a_2 = 0;

    communication_status.timestamp = 0;
    communication_status.communication_state = 0;
    communication_status.is_rc_get = false;
    communication_status.is_arduino_get = false;
    communication_status.is_wifi_get = false;
    communication_status.interrupt_time = 0;
    //publish control_mode

    vehicle_control_mode.timestamp = 0;
    vehicle_control_mode.flag_control_manual_enable = false;
    vehicle_control_mode.flag_control_auto_enable = false;
    vehicle_control_mode.flag_control_position_enable = false;
    vehicle_control_mode.flag_control_autoPilot_enable = false;
    vehicle_control_mode.flag_control_pathFollowing_enable = false;
    vehicle_control_mode.flag_control_return_enable = false;
    vehicle_control_mode.flag_control_direct_enable = false;
    vehicle_control_mode.flag_control_tack_enable = false;
    vehicle_control_mode.flag_control_jibing_enable = false;

    path_planning_mode.timestamp = 0;
    path_planning_mode.flag_planning_manual_enable = false;
    path_planning_mode.flag_planning_astar_enable = false;
    path_planning_mode.flag_planning_optimalSpeed_enable = false;
    path_planning_mode.flag_planning_direct_enable = false;
    path_planning_mode.flag_planning_avoidance_enable = false;
    path_planning_mode.flag_planning_tack_enable = false;

    //extra
    home_position.timestamp = 0;
    home_position.posX = 0;
    home_position.posY = 0;
    home_position.longitude = 0;
    home_position.latitude = 0;

    //sensor
    sensor_msg.timestamp = 0;
    sensor_msg.ux = 0;
    sensor_msg.vy = 0;
    sensor_msg.gx = 0;
    sensor_msg.gy = 0;
    sensor_msg.gz = 0;
    sensor_msg.posX = 0;
    sensor_msg.posY = 0;
    sensor_msg.roll = 0;
    sensor_msg.pitch = 0;
    sensor_msg.yaw = 0;
    sensor_msg.awa = 0;
    sensor_msg.aws = 0;

    arduino_msg.timestamp = 0;
    arduino_msg.readMark = 0;
    arduino_msg.autoFlag = 0;
    arduino_msg.motor = 0;
    arduino_msg.rudder = 0;
    arduino_msg.sail = 0;
    arduino_msg.voltage1 = 0;
    arduino_msg.voltage2 = 0;

    path_msg.timestamp = 0;
    path_msg.posX;
    path_msg.posY;
}

Commander::~Commander() {

}
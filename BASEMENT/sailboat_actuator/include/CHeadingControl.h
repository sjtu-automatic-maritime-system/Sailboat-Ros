//
// Created by hywel on 17-4-23.
//

#include "ros/ros.h"
#include "CPID.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "sailboat_message/Target_msg.h"
#include <dynamic_reconfigure/server.h>
#include "sailboat_actuator/pid_adjustment_Config.h"
#include <sstream>

#ifndef SAILBOAT_HEADINGCONTROL_H
#define SAILBOAT_HEADINGCONTROL_H


class CHeadingControl {
public:

    ros::Publisher mach_pub;

    CHeadingControl();
    CHeadingControl(double kp, double ki, double kd);
    CHeadingControl(double kp, double ki, double kd, double t, double outMax, double outMin);
    ~CHeadingControl();

    void Setting();

    double Get_Rudder();
    double Get_Sail();

    void SensorCallback(const sailboat_message::WTST_msg::ConstPtr& msg);
    void CtrlCallback(const sailboat_message::Target_msg::ConstPtr& msg);
    void PIDCallback(sailboat_actuator::pid_adjustment_Config &config, uint32_t level);

    void AP_Calc();

private:
    ros::NodeHandle ap_node;
    ros::Subscriber sensor_sub;
    ros::Subscriber ctrl_sub;

    CPID* pidp;

    double Kp;
    double Ki;
    double Kd;
    double T;
    double OutMax;
    double OutMin;

    double yawRef;
    double yawFdb;

    double rudder;
    double sail;



};


#endif //SAILBOAT_HEADINGCONTROL_H

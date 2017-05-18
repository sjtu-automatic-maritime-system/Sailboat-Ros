//
// Created by hywel on 17-4-23.
//

#include "ros/ros.h"
#include "math_tool_lib/CPID.h"
#include "mach_onboat/Mach_msg.h"
#include "sailboat_message/Sensor_msg.h"//改成处理过后的Sensor_msg
#include "sailboat_message/Target_msg.h"
#include <dynamic_reconfigure/server.h>
#include "autopilot_lib/pid_adjustment_Config.h"
#include "autopilot_lib/CSailCtrl.h"
//#include "sailboat_message/Sailboat_Simulation_msg.h"
//#include "sailboat_message/Sensor_Simulation_msg.h"
#include <sstream>

#ifndef SAILBOAT_CAUTOPILOTVER1_H
#define SAILBOAT_CAUTOPILOTVER1_H

#define pi 3.1415926

class CAutopilotVer1 {
public:
    ros::NodeHandle ap_node;
    ros::Subscriber sensor_sub;
    //ros::Subscriber sensor_simulation_sub;
    //ros::Subscriber sailboat_simulation_sub;
    ros::Subscriber ctrl_sub;
    ros::Publisher mach_pub;

    CAutopilotVer1();
    CAutopilotVer1(double kp, double ki, double kd);
    CAutopilotVer1(double kp, double ki, double kd, double t, double outMax, double outMin);
    ~CAutopilotVer1();

    //初始化
    void Init();

    //得到舵角或帆角
    double Get_Rudder();
    double Get_Sail();
    int Get_PCCtrl();
    //pid算法得到舵角
    void AP_Calc();

    //ros callback函数
    void SensorCallback(const sailboat_message::Sensor_msg::ConstPtr& msg);
    //整合后就不用了
    //void SensorSimulationCallback(const sailboat_message::Sensor_Simulation_msg::ConstPtr& msg);
    //void SailboatSimulationCallback(const sailboat_message::Sailboat_Simulation_msg::ConstPtr& msg);
    void CtrlCallback(const sailboat_message::Target_msg::ConstPtr& msg);
    void PIDCallback(autopilot_lib::pid_adjustment_Config &config, uint32_t level);



private:

    CSailCtrl sailCtrl;

    CPID* pidp;


    int PCCtrl;
    double Kp;
    double Ki;
    double Kd;
    double T;
    double OutMax;
    double OutMin;

    double yawRef;
    double yawFdb;

    double AWA;
    double AWS;

    double rudder;
    double sail;

};


#endif //SAILBOAT_CAUTOPILOTVER1_H

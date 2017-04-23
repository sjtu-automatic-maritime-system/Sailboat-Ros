//
// Created by hywel on 17-4-23.
//

#include "CHeadingControl.h"



//ros::NodeHandle ap_node;
//ros::Publisher mach_pub;
//ros::Subscriber sensor_sub;

CHeadingControl::CHeadingControl() {

    Kp = 2;
    Ki = 0;
    Kd = 0;
    T = 0.01;
    OutMax = 10;
    OutMin = -10;
    yawFdb = 0;
    yawRef = 0;

    rudder = 0;
    sail = 0;

    Setting();

}

CHeadingControl::CHeadingControl(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = 0.01;
    OutMax = 10;
    OutMin = -10;
    yawFdb = 0;
    yawRef = 0;

    rudder = 0;
    sail = 0;

    Setting();
}

CHeadingControl::CHeadingControl(double kp, double ki, double kd, double t, double outMax, double outMin) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = t;
    OutMax = outMax;
    OutMin = outMin;
    yawFdb = 0;
    yawRef = 0;

    rudder = 0;
    sail = 0;

    Setting();
}

CHeadingControl::~CHeadingControl() {

}

double CHeadingControl::Get_Rudder() {
    return rudder;
}

double CHeadingControl::Get_Sail() {
    return sail;
}

void CHeadingControl::Setting() {

    pidp=new CPID(Kp,Ki, Kd, T, OutMax,OutMin);

    mach_pub = ap_node.advertise<sailboat_message::Mach_msg>("mach", 5);

    sensor_sub = ap_node.subscribe("sensor", 2, &CHeadingControl::SensorCallback,this);
    ctrl_sub = ap_node.subscribe("targetangle", 2, &CHeadingControl::CtrlCallback,this);

}

void CHeadingControl::SensorCallback(const sailboat_message::WTST_msg::ConstPtr &msg) {
    ROS_INFO("WTST_msg sub: [%f]", msg->Yaw);
    yawFdb = msg->Yaw;
}

void CHeadingControl::CtrlCallback(const sailboat_message::Target_msg::ConstPtr &msg) {
    ROS_INFO("Tatget_msg sub: [%f]",msg->TargetAngle);
    yawRef = msg->TargetAngle;
}

void CHeadingControl::PIDCallback(sailboat_actuator::pid_adjustment_Config &config, uint32_t level) {
    ROS_INFO("PID_ad config: [%f] [%f] [%f]",config.Kp,config.Ki,config.Kd);
    Kp = config.Kp;
    Ki = config.Ki;
    Kd = config.Kd;
    pidp->Set_Kp(Kp);
    pidp->Set_Ki(Ki);
    pidp->Set_Kd(Kd);
    double p,i,d = pidp->Get_Kpid();
    ROS_INFO("CPID: [%f] [%f] [%f]",p,i,d);
}

void CHeadingControl::AP_Calc() {
    pidp->Set_Ref(yawRef);
    pidp->Set_Fdb(yawFdb);
    rudder = pidp->PID_Calc();
}
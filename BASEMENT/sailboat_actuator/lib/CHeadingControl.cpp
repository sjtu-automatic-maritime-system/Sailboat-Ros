//
// Created by hywel on 17-4-23.
//

#include "CHeadingControl.h"



//ros::NodeHandle ap_node;
//ros::Publisher mach_pub;
//ros::Subscriber sensor_sub;

CHeadingControl::CHeadingControl() {

    Kp = 0.5;
    Ki = 0;
    Kd = 0;
    T = 0.1;
    OutMax = 0.52;
    OutMin = -0.52;
    yawFdb = 0;
    yawRef = 0;

    rudder = 0;
    sail = 0;

    Init();

}

CHeadingControl::CHeadingControl(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    T = 0.1;
    OutMax = 0.52;
    OutMin = -0.52;
    yawFdb = 0;
    yawRef = 0;

    rudder = 0;
    sail = 0;

    Init();
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

    Init();
}

CHeadingControl::~CHeadingControl() {
    delete pidp;

}

double CHeadingControl::Get_Rudder() {
    return rudder;
}

double CHeadingControl::Get_Sail() {
    return sail;
}

void CHeadingControl::Init() {

    pidp=new CPID(Kp,Ki, Kd, T, OutMax,OutMin);

    mach_pub = ap_node.advertise<sailboat_message::Mach_msg>("mach", 5);

    sensor_sub = ap_node.subscribe("sensor", 5, &CHeadingControl::SensorCallback,this);
    //sensor_simulation_sub = ap_node.subscribe("sensor", 2, &CHeadingControl::SensorSimulationCallback,this);
    //sailboat_simulation_sub = ap_node.subscribe("sailboat", 2, &CHeadingControl::SailboatSimulationCallback,this);
    ctrl_sub = ap_node.subscribe("targetangle", 5, &CHeadingControl::CtrlCallback,this);

}

void CHeadingControl::SensorCallback(const sailboat_message::Sensor_msg::ConstPtr &msg) {
    ROS_INFO("I get AWA: [%f]", msg->AWA);
    yawFdb = msg->Yaw;
    AWA = msg->AWA;
    AWS = msg->AWS;
}


//void CHeadingControl::SensorSimulationCallback(const sailboat_message::Sensor_Simulation_msg::ConstPtr& msg){

//}

//void CHeadingControl::SailboatSimulationCallback(const sailboat_message::Sailboat_Simulation_msg::ConstPtr& msg) {
//    ROS_INFO("sailboat_msg sub: [%f]", msg->yaw);
//    yawFdb = msg->yaw;
//}


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
    ROS_INFO("YAWRef and YAWFdb: [%f] [%f]",yawRef,yawFdb);
    rudder = pidp->PID_Calc()*180/pi;

    sail = -5.0/6*AWA*180/pi;
    if (sail> 90)
        sail = 90;
    else if (sail < -90)
        sail = -90;
    ROS_INFO("sail: [%f]",sail);

}
//
// Created by hywel on 17-4-23.
//

#include "autopilot/CAutopilotVer1.h"


#include "ros/ros.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/Sensor_msg.h"//改成处理过后的Sensor_msg
#include "sailboat_message/Target_msg.h"
#include <dynamic_reconfigure/server.h>
#include "autopilot/pid_adjustment_Config.h"


//Create an object of class SubscribeAndPublish that will take care of everything
static CAutopilotVer1 autopilotver1(0.5,0,0,0.1,0.52,-0.52);

void SensorCallback(const sailboat_message::Sensor_msg::ConstPtr &msg) {
    ROS_INFO("I get AWA: [%f]", msg->AWA);
    autopilotver1.yawFdb = msg->Yaw;
    autopilotver1.AWA = msg->AWA;
    autopilotver1.AWS = msg->AWS;
}

void CtrlCallback(const sailboat_message::Target_msg::ConstPtr &msg) {
    ROS_INFO("Tatget_msg sub: [%f]",msg->TargetAngle);
    autopilotver1.yawRef = msg->TargetAngle;
}

void PIDCallback(autopilot::pid_adjustment_Config &config, uint32_t level) {
    ROS_INFO("PID_ad config: [%f] [%f] [%f]",config.Kp,config.Ki,config.Kd);
    autopilotver1.Kp = config.Kp /10;
    autopilotver1.Ki = config.Ki /10;
    autopilotver1.Kd = config.Kd /10;
    autopilotver1.set_Pid();

    if (config.PC_Ctrl == true)
        autopilotver1.PCCtrl = 1;
    else
        autopilotver1.PCCtrl = 0;

}

void Mach_pub(sailboat_message::Mach_msg& msg){
    msg.timestamp = ros::Time::now().toSec();
    msg.motor = 50;
    msg.rudder = autopilotver1.rudder;
    msg.sail = autopilotver1.sail;
    msg.PCCtrl = autopilotver1.PCCtrl;
    //ROS_INFO("I talk Rudder_Angle: [%f]", msg.rudder);
    //ROS_INFO("I talk Sail_Angle: [%f]", msg.sail);
}


int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "autopilot_ver1");
    ros::NodeHandle ap_node;
    ros::Subscriber sensor_sub;
    //ros::Subscriber sensor_simulation_sub;
    //ros::Subscriber sailboat_simulation_sub;
    ros::Subscriber ctrl_sub;
    ros::Publisher mach_pub;

    mach_pub = ap_node.advertise<sailboat_message::Mach_msg>("mach", 5);

    sensor_sub = ap_node.subscribe("sensor", 2, SensorCallback);
    //sensor_simulation_sub = ap_node.subscribe("sensor", 2, &CHeadingControl::SensorSimulationCallback,this);
    //sailboat_simulation_sub = ap_node.subscribe("sailboat", 2, &CHeadingControl::SailboatSimulationCallback,this);
    ctrl_sub = ap_node.subscribe("targetangle", 2, CtrlCallback);



    dynamic_reconfigure::Server<autopilot::pid_adjustment_Config> srv;
    dynamic_reconfigure::Server<autopilot::pid_adjustment_Config>::CallbackType f;

    f = boost::bind(&PIDCallback,  _1, _2);
    srv.setCallback(f);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        autopilotver1.AP_Calc(); //计算

        sailboat_message::Mach_msg msg;
        Mach_pub(msg);
        mach_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

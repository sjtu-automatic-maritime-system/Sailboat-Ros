//
// Created by hywel on 17-5-2.
//

#include "sailboat_simulation_lib/CSimulationVer1.h"
//ros
#include "ros/ros.h"
#include "sailboat_message/Target_msg.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include "sailboat_message/Mach_msg.h"

static CSimulationVer1 SME;

void WindCallback(const sailboat_message::Wind_Simulation_msg::ConstPtr &msg) {
    ROS_INFO("Wind_msg sub: [%f] [%f]", msg->TWA , msg->TWS);
    SME.windDirection = msg->TWA;
    SME.windVelocity = msg->TWS;
}

void MachCallback(const sailboat_message::Mach_msg::ConstPtr &msg) {
    ROS_INFO("Mach_msg sub: [%f] [%f]", msg->rudder , msg->sail);
    SME.rudderAngle = msg->rudder;
    SME.sailAngle = msg->sail;
    if(SME.sailAngle>1.5) SME.sailAngle=1.5;
    if(SME.sailAngle<-1.5) SME.sailAngle=-1.5;
    SME.delta_r = SME.rudderAngle;
    SME.delta_s = SME.sailAngle;
}

void Sensor_pub(sailboat_message::Sensor_msg& msg){

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.ux = SME.uu;
    msg.vy = SME.vv;
    msg.gx = SME.pp;
    msg.gz = SME.rr;
    msg.Posx = SME.XX;
    msg.Posy = SME.YY;
    msg.Roll = SME.phi;
    msg.Yaw = SME.psi;
    msg.AWA = SME.AWA;
    msg.AWS = SME.AWS;

    ROS_INFO("I talk Sailboat_Simulation: [%f] [%f]", msg.Posx, msg.Posy);
}


int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "simulation_ver1");
    ros::NodeHandle simulation_node;
    ros::Subscriber wind_sub;
    ros::Subscriber mach_sub;
    ros::Publisher sensor_pub;
    //sailboat_message::Sailboat_Simulation_msg ssmsg;

    mach_sub = simulation_node.subscribe("mach", 2, MachCallback);
    wind_sub = simulation_node.subscribe("wind", 2, WindCallback);

    sensor_pub = simulation_node.advertise<sailboat_message::Sensor_msg>("sensor", 5);

    //显示仿真数据
    SME.ShowData();
    //SME.HideData();

    ros::Rate loop_rate(50);

    //设置帆船初始状态
    SME.SettingAttitudeInit(0,0,0,0,0,0,0,0);

    while (ros::ok())
    {
        SME.Sailboat_Calc(0.02);


        sailboat_message::Sensor_msg msg;
        Sensor_pub(msg);
        sensor_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

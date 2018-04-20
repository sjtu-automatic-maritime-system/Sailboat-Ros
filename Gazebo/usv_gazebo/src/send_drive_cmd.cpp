//
// Created by hywel on 4/20/18.
//

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include <usv_gazebo_plugins/Drive.h>

#include <dynamic_reconfigure/server.h>

double Rudder;
double TWS;
bool get_mach = false;

void machCallback(const sailboat_message::Mach_msg::ConstPtr& msg){
    //ROS_INFO("get mach");
    get_mach = true;
    Rudder = msg->rudder;
}

void windCallback(const sailboat_message::Wind_Simulation_msg::ConstPtr& msg){
    TWS = msg->TWS;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_drive_cmd");

    ros::NodeHandle nh;
    ros::Publisher drive_cmd_pub;

    drive_cmd_pub = nh.advertise<usv_gazebo_plugins::Drive>("cmd_drive", 2);

    ros::Subscriber mach_sub = nh.subscribe("mach", 2, &machCallback);
    ros::Subscriber wind_sub = nh.subscribe("wind", 2, &windCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (!get_mach) {
            ROS_INFO("didn't get mach msg");
            continue;
        }

        usv_gazebo_plugins::Drive msg;
        
        msg.left =  TWS + Rudder*TWS/3;
        msg.right = TWS - Rudder*TWS/3;
        drive_cmd_pub.publish(msg);
        

    }

    return 0;
}

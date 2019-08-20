#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/Arduino_msg.h"
#include <cmath>

bool dxl_native = false;

int autoFlag;
double baseSailAngle;
double baseRudderAngle;
double baseSailAngleLast;
double baseRudderAngleLast;
double baseMotor;

int readMark;
int rcCtrl;
double arduinoSailAngle;
double arduinoRudderAngle;
double arduinoMotor;

int rosCtrl;
double rosSailAngle;
double rosRudderAngle;
double rosMotor;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


void arduinoMachCallback(const sailboat_message::Arduino_msg::ConstPtr& msg)
{  
    readMark = int(msg->readMark);
    rcCtrl = 1 - int(msg->autoFlag);
    arduinoSailAngle = msg->sail * 3.14 / 180;
    arduinoRudderAngle = msg->rudder * 3.14 / 180;
    arduinoMotor = msg->motor;
}

void rosMachCallback(const sailboat_message::Mach_msg::ConstPtr& msg)
{
    rosCtrl = int(msg->PCCtrl);
    rosMotor = msg->motor;
    rosSailAngle = msg->sail;
    rosRudderAngle = msg->rudder;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_control");

    ros::NodeHandle n;

    ros::Subscriber rosMachSub = n.subscribe("/mach", 10, rosMachCallback);
    ros::Subscriber arduinoSub = n.subscribe("/arduino", 10, arduinoMachCallback);

    ros::Publisher baseMachPub = n.advertise<sailboat_message::Mach_msg>("/base/mach", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (readMark == 0){
            ROS_WARN("error in arduino common");
        }
        if (rcCtrl == 1){
            //remote control
            baseSailAngle = fabs(arduinoSailAngle);
            baseRudderAngle = arduinoRudderAngle;
            baseMotor = arduinoMotor;
            autoFlag = 0;
        }
        else{
            if (rosCtrl == 1){
                //ros control
                baseSailAngle = fabs(rosSailAngle);
                baseRudderAngle = rosRudderAngle;
                baseMotor = rosMotor;
                autoFlag = 1;
            }
            else{
                //rcCtrl
                baseSailAngle = fabs(arduinoSailAngle);
                baseRudderAngle = arduinoRudderAngle;
                baseMotor = arduinoMotor;
                autoFlag = 0;
            }
        }

        sailboat_message::Mach_msg base_mach;
        base_mach.header.stamp = ros::Time::now();
        base_mach.header.frame_id = "base_link";
        base_mach.motor = baseMotor;
        base_mach.rudder = baseRudderAngle;
        base_mach.sail = baseSailAngle;
        base_mach.PCCtrl = autoFlag;

        baseMachPub.publish(base_mach);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}


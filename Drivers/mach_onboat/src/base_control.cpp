#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sailboat_message/Mach_msg.h"
#include "dynamixel_workbench_msgs/JointCommand.h"
#include "sailboat_message/Dxl_Control_srv.h"
#include "sailboat_message/Dxl_State_srv.h"
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
    ros::ServiceClient DyClient1 = n.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");
    ros::ServiceClient DyClient2 = n.serviceClient<sailboat_message::Dxl_Control_srv>("/dxl_control_srv");
    

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

        double sailAngle = baseSailAngle*4 - 3.14;

        if (sailAngle > 3.14){
            sailAngle = 3.14;
        }
        if (sailAngle < -3.14){
            sailAngle = -3.14;
        }
        if (baseRudderAngle > 0.87){
            baseRudderAngle = 0.87;
        }
        if (baseRudderAngle < -0.87){
            baseRudderAngle = -0.87;
        }
        
        if (dxl_native){
            dynamixel_workbench_msgs::JointCommand srvSail;
            srvSail.request.unit = "rad";
            srvSail.request.id = 1;
            srvSail.request.goal_position = sailAngle;
            if (DyClient1.call(srvSail))
            {
                ROS_INFO("sail contrl result = %d", srvSail.response.result);
            }
            else
            {
                ROS_ERROR("Failed to call service joint_command");
            }

            dynamixel_workbench_msgs::JointCommand srvRudder;
            srvRudder.request.unit = "rad";
            srvRudder.request.id = 2;
            srvRudder.request.goal_position = baseRudderAngle;
            if (DyClient1.call(srvRudder))
            {
                ROS_INFO("rudder contrl result = %d", srvRudder.response.result);
            }
            else
            {
                ROS_ERROR("Failed to call service joint_command");
            }
        }else{
            sailboat_message::Dxl_Control_srv srvSail;
            srvSail.request.dxl_id = 1;
            srvSail.request.position = sailAngle;
            if (DyClient2.call(srvSail))
            {
                ROS_INFO("sail contrl result = %d", srvSail.response.state);
            }
            else
            {
                ROS_ERROR("Failed to call service joint_command");
            }

            sailboat_message::Dxl_Control_srv srvRudder;
            srvRudder.request.dxl_id = 2;
            srvRudder.request.position = baseRudderAngle;
            if (DyClient2.call(srvRudder))
            {
                ROS_INFO("rudder contrl result = %d", srvRudder.response.state);
            }
            else
            {
                ROS_ERROR("Failed to call service joint_command");
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


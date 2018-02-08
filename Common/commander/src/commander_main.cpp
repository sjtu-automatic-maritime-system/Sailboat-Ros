//
// Created by hywel on 11/9/17.
//

#include <commander/commander.h>

#include "ros/ros.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"

#include <dynamic_reconfigure/server.h>


static Commander commander;

int main(){

    ROS_INFO("commander start!!!");
    return 0;
}
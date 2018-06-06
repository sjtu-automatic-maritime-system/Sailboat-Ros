//by Boxian Deng, Jun 03, 2018
//used to publish static messages
//abandoned now
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/obs_msg.h"

#include "avoidance_v2/avoidance_v2.h"

#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "obs_msg");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sailboat_message::obs_msg>("obs_msg", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    sailboat_message::obs_msg msg;

    //old version of obstacle information publisher, now publish obstacle information at detection_mock.cpp
    /*
    double array[8] ={INFINITY_DOUBLE, INFINITY_DOUBLE , INFINITY_DOUBLE, INFINITY_DOUBLE ,INFINITY_DOUBLE ,INFINITY_DOUBLE, INFINITY_DOUBLE, INFINITY_DOUBLE};
    for (int i=0;i < 8; i++){
      msg.data.push_back(array[i]);
    }
    */

    //msg.wind_angle = 60;
    //msg.wind_speed = 5;

    msg.target_angle = 30 / 180 * PI;

    //ROS_INFO("[Publisher from obs_msg] Wind information: wind angle is %f and wind speed is %f", msg.wind_angle, msg.wind_speed);
    ROS_INFO("[Publisher from obs_msg] Target information: target angle is %f", msg.target_angle);


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}


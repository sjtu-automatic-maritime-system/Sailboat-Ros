//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include <sstream>

#define pi 3.1415926

int main(int argc, char **argv)
{

  ros::init(argc, argv, "wind_simulation_talker");
  ros::NodeHandle n;
  ros::Publisher Wind_pub = n.advertise<sailboat_message::Wind_Simulation_msg>("wind", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    //float64 WindAngle
    sailboat_message::Wind_Simulation_msg msg;
    msg.TWA = pi/4;
    msg.TWS = 3;
    ROS_INFO("I talk WindAngle: [%f]", msg.TWA);
    Wind_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

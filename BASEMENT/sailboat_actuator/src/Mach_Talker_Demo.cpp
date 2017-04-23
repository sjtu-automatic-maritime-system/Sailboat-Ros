//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Mach_msg.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Mach_talker");
  ros::NodeHandle n;
  ros::Publisher Mach_pub = n.advertise<sailboat_message::Mach_msg>("mach", 5);
  ros::Rate loop_rate(20);

  int num = 0;

  while (ros::ok())
  {
    //float64 motor    0-200
    //float64 rudder   50-130
    //float64 sail     50-130
    sailboat_message::Mach_msg msg;
    
    msg.motor = num;
    msg.rudder = 90;
    msg.sail = 90;

    ROS_INFO("I talk: [%f] [%f] [%f]", msg.motor, msg.rudder, msg.sail );

    Mach_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    num++;
  }

  return 0;
}

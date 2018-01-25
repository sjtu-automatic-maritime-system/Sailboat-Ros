//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Target_msg.h"
#include <dynamic_reconfigure/server.h>
#include "environment_simulation/sailboat_targetangle_simulation_Config.h"
#include <sstream>

static float target = 0;


void callback(environment_simulation::sailboat_targetangle_simulation_Config &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %d", 
    //       config.Sail_Angle,config.Rudder_Angle, 
    //       config.size);
  target = config.targetAngle;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_simulation_talker");
  ros::NodeHandle n;
  ros::Publisher target_pub = n.advertise<sailboat_message::Target_msg>("targetangle", 10);

  dynamic_reconfigure::Server<environment_simulation::sailboat_targetangle_simulation_Config> dserver;
  dynamic_reconfigure::Server<environment_simulation::sailboat_targetangle_simulation_Config>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  dserver.setCallback(f);

  ROS_INFO("Spinning node");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

  //   // int16 MachFlag
  //   // float64 motor
  //   // float64 rudder
  //   // float64 sail
    sailboat_message::Target_msg msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.TargetAngle = target;
    
    ROS_INFO("I talk TargetAngle: [%f]", msg.TargetAngle);
    target_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

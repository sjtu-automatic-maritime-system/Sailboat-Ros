//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Mach_msg.h"
#include <dynamic_reconfigure/server.h>
#include "environment_simulation/sailboat_mach_simulation_Config.h"
#include <sstream>

static float motor = 50;
static float rudder = 0;
static float sail = 0;
static int PCCtrl = 0;
static int is_sim = 0;
void callback(environment_simulation::sailboat_mach_simulation_Config &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %d", 
    //       config.Sail_Angle,config.Rudder_Angle, 
    //       config.size);
  motor = config.motor;
  rudder = config.Rudder_Angle/57.3;
  sail = config.Sail_Angle/57.3;
  if (config.PC_Ctrl)
    PCCtrl = 1;
  else
    PCCtrl = 0;

  if (config.is_sim)
    is_sim = 1;
  else
    is_sim = 0;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mach_simulation_talker");
  ros::NodeHandle n;
  ros::Publisher Mach_pub = n.advertise<sailboat_message::Mach_msg>("mach", 10);

  dynamic_reconfigure::Server<environment_simulation::sailboat_mach_simulation_Config> dserver;
  dynamic_reconfigure::Server<environment_simulation::sailboat_mach_simulation_Config>::CallbackType f;
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
    if (is_sim == 1){    
      sailboat_message::Mach_msg msg;
      msg.motor = motor;
      msg.rudder = rudder;
      msg.sail = sail;
      msg.PCCtrl = PCCtrl;
      
      //ROS_INFO("I talk Rudder_Angle: [%f]", msg.rudder);
      Mach_pub.publish(msg);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

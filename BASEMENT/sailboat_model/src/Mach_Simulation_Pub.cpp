#include "ros/ros.h"
#include "sailboat_message/Mach_msg.h"
#include <dynamic_reconfigure/server.h>
#include "sailboat_model/sailboat_mach_simulation_Config.h"
#include <sstream>

static float motor = 0;
static float rudder = 0;
static float sail = 0;

void callback(sailboat_model::sailboat_mach_simulation_Config &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %d", 
    //       config.Sail_Angle,config.Rudder_Angle, 
    //       config.size);
  rudder = config.Rudder_Angle;
  sail = config.Sail_Angle;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Mach_Simulation_Talker");
  ros::NodeHandle n;
  ros::Publisher Mach_pub = n.advertise<sailboat_message::Mach_msg>("Mach", 10);

  dynamic_reconfigure::Server<sailboat_model::sailboat_mach_simulation_Config> server;
  dynamic_reconfigure::Server<sailboat_model::sailboat_mach_simulation_Config>::CallbackType f;

  ROS_INFO("Spinning node");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


  //   // int16 MachFlag
  //   // float64 motor
  //   // float64 rudder
  //   // float64 sail
    sailboat_message::Mach_msg msg;
    msg.MachFlag = 0;
    msg.motor = 0;
    msg.rudder = rudder;
    msg.sail = sail;

    ROS_INFO("I talk: [%f]", msg.rudder);
    Mach_pub.publish(msg);



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

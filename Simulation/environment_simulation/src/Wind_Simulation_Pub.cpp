//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include <dynamic_reconfigure/server.h>
#include "environment_simulation/wind_simulation_Config.h"
#include <sstream>

#define PI 3.1415926

static float TWA = 0;
static float TWS = 3;

void callback(environment_simulation::wind_simulation_Config &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %d",
  //       config.Sail_Angle,config.Rudder_Angle,
  //       config.size);
  TWA = config.TWA/57.3;
  TWS = config.TWS;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "wind_simulation_talker");
  ros::NodeHandle n;
  ros::Publisher Wind_pub = n.advertise<sailboat_message::Wind_Simulation_msg>("wind", 10);

  dynamic_reconfigure::Server<environment_simulation::wind_simulation_Config> dserver;
  dynamic_reconfigure::Server<environment_simulation::wind_simulation_Config>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  dserver.setCallback(f);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    //float64 WindAngle
    sailboat_message::Wind_Simulation_msg msg;
    msg.TWA = TWA;
    msg.TWS = TWS;
    //ROS_INFO("I talk WindAngle: [%f]", msg.TWA);
    Wind_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//
// File: ert_main.cpp
//
// Code generated for Simulink model 'race_course'.
//
// Model version                  : 1.311
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Thu Jul 06 13:08:27 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "fleet_race/race_course.h"               // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/fleet_race_out.h"

#include <dynamic_reconfigure/server.h>
#include "fleet_race/fleet_race_Config.h"

static race_courseModelClass race_course_Obj;// Instance of model class
bool pcCtrl;
//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = 0;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(race_course_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  race_course_Obj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void callback(const sailboat_message::Sensor_msg::ConstPtr msg) {
  race_course_Obj.race_course_U.North = msg->Posx;
  race_course_Obj.race_course_U.East = msg->Posy;
  race_course_Obj.race_course_U.Roll = msg->Roll;
  race_course_Obj.race_course_U.Yaw = msg->Yaw;
  race_course_Obj.race_course_U.Roll_rate = msg->gx;
  race_course_Obj.race_course_U.Yaw_rate = msg->gz;
  race_course_Obj.race_course_U.Airmar_wind_angle = msg->AWA;
  race_course_Obj.race_course_U.Airmar_wind_speed = msg->AWS;
}

void FleetraceCfgcallback(fleet_race::fleet_race_Config &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f",config.Kp,config.Ki,config.Kd);

  race_course_Obj.race_course_P.Kp = config.Kp;
  race_course_Obj.race_course_P.Ki = config.Ki;
  race_course_Obj.race_course_P.Kd = config.Kd;
  race_course_Obj.race_course_P.R_reach = config.R_reach;
  race_course_Obj.race_course_P.R_reach_big = config.R_reach_big;
  race_course_Obj.race_course_P.extra_leg_len = config.extra_leg_len;

  race_course_Obj.race_course_P.max_loose_time = config.max_loose_time;
  race_course_Obj.race_course_P.max_roll_allowed = config.max_roll_allowed;
  race_course_Obj.race_course_P.pos_history_len = config.pos_history_len;
  race_course_Obj.race_course_P.run_period = config.run_period;
  race_course_Obj.race_course_P.ship_speed_history_len = config.ship_speed_history_len;
  race_course_Obj.race_course_P.upwind_leg = config.upwind_leg;
  race_course_Obj.race_course_P.wind_mean_time = config.wind_mean_time;

  race_course_Obj.race_course_P.race_points[0] = config.point0_x;
  race_course_Obj.race_course_P.race_points[1] = config.point1_x;
  race_course_Obj.race_course_P.race_points[2] = config.point2_x;
  race_course_Obj.race_course_P.race_points[3] = config.point3_x;

  race_course_Obj.race_course_P.race_points[4] = config.point0_y;
  race_course_Obj.race_course_P.race_points[5] = config.point1_y;
  race_course_Obj.race_course_P.race_points[6] = config.point2_y;
  race_course_Obj.race_course_P.race_points[7] = config.point3_y;
}

void getOutMachPut(sailboat_message::Mach_msg& msg){

  msg.timestamp = ros::Time::now().toSec();
  msg.motor = 0;
  msg.rudder = race_course_Obj.race_course_Y.Rudder;
  msg.sail = race_course_Obj.race_course_Y.Sail;

  msg.PCCtrl = pcCtrl;

}

void getOutput(sailboat_message::fleet_race_out& msg){

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.Sail = race_course_Obj.race_course_Y.Sail;
  msg.Rudder = race_course_Obj.race_course_Y.Rudder;
  msg.leg = race_course_Obj.race_course_Y.leg;
  msg.los_heading = race_course_Obj.race_course_Y.los_heading;
  msg.speed_angle = race_course_Obj.race_course_Y.speed_angle;
  msg.speed_angle_d = race_course_Obj.race_course_Y.speed_angle_d;
  msg.wind_speed = race_course_Obj.race_course_Y.wind_speed;
  msg.wind_angle_ground = race_course_Obj.race_course_Y.wind_angle_ground;
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustates how you do this relative to initializing the model.
//
int_T main(int_T argc, char **argv)
{
  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Initialize model
  race_course_Obj.initialize();

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.1 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep();

  ros::init(argc, argv, "scanning");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher fleet_race_pub;
  ros::Publisher mach_pub;
  fleet_race_pub = nh.advertise<sailboat_message::fleet_race_out>("fleet_race_out", 2);
  mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

  dynamic_reconfigure::Server<fleet_race::fleet_race_Config> server;
  dynamic_reconfigure::Server<fleet_race::fleet_race_Config>::CallbackType f;

  f = boost::bind(&FleetraceCfgcallback, _1, _2);
  server.setCallback(f);

  sub = nh.subscribe("sensor", 100, callback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rt_OneStep();
    sailboat_message::fleet_race_out msg;
    sailboat_message::Mach_msg msgMach;
    getOutMachPut(msgMach);
    getOutput(msg);
    fleet_race_pub.publish(msg);
    mach_pub.publish(msgMach);

    loop_rate.sleep();
  }



  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(race_course_Obj.getRTM()) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  race_course_Obj.terminate();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

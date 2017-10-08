//
// File: ert_main.cpp
//
// Code generated for Simulink model 'sailboat_ap_ok'.
//
// Model version                  : 1.221
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Tue Sep 26 11:36:58 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "spare_function/sailboat_ap_ok.h"            // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "spare_function/spare_function_Config.h"
#include "spare_function/spare_function_out.h"
#include "spare_function/spare_function_para.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"
#include <dynamic_reconfigure/server.h>
#include <spare_function/sailboat_ap_ok.h>

static sailboat_ap_okModelClass rtObj; // Instance of model class

int pcCtrl = 0;
ros::Publisher spare_function_para_pub;

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
    rtmSetErrorStatus(rtObj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  rtObj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void getInput(const sailboat_message::Sensor_msg::ConstPtr msg) {
  rtObj.rtU.roll = msg->Roll;
  rtObj.rtU.yaw = msg->Yaw;
  rtObj.rtU.relative_wind = msg->AWA;
}

void getPara(spare_function::spare_function_Config &config){
  rtObj.rtP.oyaw    = config.oyaw;
  rtObj.rtP.rudderP = config.rudderP;
  rtObj.rtP.rudderI = config.rudderI;
  rtObj.rtP.rudderD = config.rudderD;
  rtObj.rtP.sailP   = config.sailP;
  rtObj.rtP.sailI   = config.sailI;
  rtObj.rtP.sailD   = config.sailD;
}

void getOutMachPut(sailboat_message::Mach_msg &msg) {
  msg.timestamp = ros::Time::now().toSec();
  msg.motor = 0;
  msg.rudder = rtObj.rtY.Rudder;
  msg.sail   = rtObj.rtY.Sail;
  msg.PCCtrl = pcCtrl;
}

void getOutput(spare_function::spare_function_out &msg) {

  msg.rudder = rtObj.rtY.Rudder;
  msg.sail = rtObj.rtY.Sail;
}

void getOutParaPut(spare_function::spare_function_para &msg) {
  msg.oyaw    = rtObj.rtP.oyaw    ;
  msg.rudderP= rtObj.rtP.rudderP;
  msg.rudderI= rtObj.rtP.rudderI;
  msg.rudderD= rtObj.rtP.rudderD;
  msg.sailP  = rtObj.rtP.sailP  ;
  msg.sailI  = rtObj.rtP.sailI  ;
  msg.sailD  = rtObj.rtP.sailD  ;
}

void cfgCallback(spare_function::spare_function_Config &config, uint32_t level) {
//ROS_INFO("Reconfigure Request: %f %f %f", config.Kp, config.Ki, config.Kd);

  if (config.PC_Ctrl == true)
    pcCtrl = 1;
  else
    pcCtrl = 0;
  getPara(config);
}

void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr msg) {
  getInput(msg);
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
  rtObj.initialize();

  ros::init(argc, argv, "scanning");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher spare_function_pub;

  ros::Publisher mach_pub;
  spare_function_pub = nh.advertise<spare_function::spare_function_out>("spare_function_out", 2);
  spare_function_para_pub = nh.advertise<spare_function::spare_function_para>("spare_function_para", 2);
  mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

  dynamic_reconfigure::Server<spare_function::spare_function_Config> server;
  dynamic_reconfigure::Server<spare_function::spare_function_Config>::CallbackType f;

  f = boost::bind(&cfgCallback, _1, _2);
  server.setCallback(f);

//    dynamic_reconfigure::Server<scanning::pcCtrl_Config> server2;
//    dynamic_reconfigure::Server<scanning::pcCtrl_Config>::CallbackType f2;
//
//    f2 = boost::bind(&PcCtrlCfgcallback, _1, _2);
//    server2.setCallback(f2);

  sub = nh.subscribe("sensor", 100, sensorCallback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rt_OneStep();
    spare_function::spare_function_out msg;
    sailboat_message::Mach_msg msgMach;
    spare_function::spare_function_para msgPara;

    getOutMachPut(msgMach);
    getOutput(msg);
    getOutParaPut(msgPara);

    spare_function_pub.publish(msg);
    spare_function_para_pub.publish(msgPara);
    mach_pub.publish(msgMach);

    loop_rate.sleep();
  }
  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.1 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep();

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(rtObj.getRTM()) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  rtObj.terminate();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

//
// File: ert_main.cpp
//
// Code generated for Simulink model 'scanning'.
//
// Model version                  : 1.230
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Fri Jun 30 13:51:12 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush
#include "station_keeping/keeping.h"
#include "keeping.h"           // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/station_keeping_out.h"
#include "sailboat_message/station_keeping_para.h"
#include "sailboat_message/Mach_msg.h"

#include <dynamic_reconfigure/server.h>

#include "station_keeping/station_keeping_Config.h"


int pcCtrl = 0;

static keepingModelClass station_keeping_Obj;// Instance of model class

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep i
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
        rtmSetErrorStatus(station_keeping_Obj.getRTM(), "Overrun");
        return;
    }

    OverrunFlag = true;

    // Save FPU context here (if necessary)
    // Re-enable timer or interrupt here
    // Set model inputs here

    // Step the model
    station_keeping_Obj.step();

    // Get model outputs here

    // Indicate task complete
    OverrunFlag = false;

    // Disable interrupts here
    // Restore FPU context here (if necessary)
    // Enable interrupts here
}



void getInput(const sailboat_message::Sensor_msg::ConstPtr msg) {
    station_keeping_Obj.keeping_U.north = msg->Posx;
    station_keeping_Obj.keeping_U.east = msg->Posy;
    station_keeping_Obj.keeping_U.roll = msg->Roll;
    station_keeping_Obj.keeping_U.yaw = msg->Yaw;
    station_keeping_Obj.keeping_U.roll_rate = msg->gx;
    station_keeping_Obj.keeping_U.yaw_rate = msg->gz;
    station_keeping_Obj.keeping_U.Airmar_wind_angle = msg->AWA;
    station_keeping_Obj.keeping_U.Airmar_wind_speed = msg->AWS;
}

void ScanningCfgcallback(station_keeping::station_keeping_Config &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f",config.Kp,config.Ki,config.Kd);
    if (config.PC_Ctrl == true)
        pcCtrl = 1;
    else
        pcCtrl = 0;
    station_keeping_Obj.keeping_P.Kp = config.Kp;
    station_keeping_Obj.keeping_P.Ki = config.Ki;
    station_keeping_Obj.keeping_P.Kd = config.Kd;
    station_keeping_Obj.keeping_P.max_loose_time = config.max_loose_time;
    station_keeping_Obj.keeping_P.max_roll_allowed = config.max_roll_allowed;
    station_keeping_Obj.keeping_P.pos_history_len = config.pos_history_len;
    station_keeping_Obj.keeping_P.run_period = config.run_period;
    station_keeping_Obj.keeping_P.ship_speed_history_len = config.ship_speed_history_len;
    station_keeping_Obj.keeping_P.tacking_force_discount = config.tacking_force_discount;
    station_keeping_Obj.keeping_P.wind_mean_time = config.wind_mean_time;

    station_keeping_Obj.keeping_P.jibing_time  = config.jibing_time  ;
    station_keeping_Obj.keeping_P.tacking_time = config.tacking_time ;

    station_keeping_Obj.keeping_P.point_keeping[0] = config.point_keeping_x;
    station_keeping_Obj.keeping_P.point_keeping[1] = config.point_keeping_y;


}



void callback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    getInput(msg);
}

void getOutMachPut(sailboat_message::Mach_msg& msg){

    msg.timestamp = ros::Time::now().toSec();
    msg.motor = 50;
    msg.rudder = station_keeping_Obj.keeping_Y.rudder;
    msg.sail = station_keeping_Obj.keeping_Y.sail;

    msg.PCCtrl = pcCtrl;

}

void getOutput(sailboat_message::station_keeping_out& msg){

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.sail = station_keeping_Obj.keeping_Y.sail;
    msg.rudder = station_keeping_Obj.keeping_Y.rudder;
    msg.los_heading = station_keeping_Obj.keeping_Y.los_heading;
    msg.speed_angle = station_keeping_Obj.keeping_Y.speed_angle;
    msg.speed_angle_d = station_keeping_Obj.keeping_Y.speed_angle_d;
    msg.wind_speed = station_keeping_Obj.keeping_Y.wind_speed;
    msg.wind_angle_ground = station_keeping_Obj.keeping_Y.wind_angle_ground;

}

void getOutParaPut(sailboat_message::station_keeping_para &msg)
{
    msg.Kp = station_keeping_Obj.keeping_P.Kp;
    msg.Ki = station_keeping_Obj.keeping_P.Ki;
    msg.Kd = station_keeping_Obj.keeping_P.Kd;

    msg.max_loose_time =   station_keeping_Obj.keeping_P.max_loose_time;
    msg.max_roll_allowed = station_keeping_Obj.keeping_P.max_roll_allowed;
    msg.pos_history_len =  station_keeping_Obj.keeping_P.pos_history_len;
    msg.run_period =       station_keeping_Obj.keeping_P.run_period;
    msg.ship_speed_history_len = station_keeping_Obj.keeping_P.ship_speed_history_len;
    msg.tacking_force_discount = station_keeping_Obj.keeping_P.tacking_force_discount;
    msg.wind_mean_time =         station_keeping_Obj.keeping_P.wind_mean_time;

    msg.jibing_time  = station_keeping_Obj.keeping_P.jibing_time  ;
    msg.tacking_time = station_keeping_Obj.keeping_P.tacking_time ;

    msg.point_keeping_x = station_keeping_Obj.keeping_P.point_keeping[0];
    msg.point_keeping_y = station_keeping_Obj.keeping_P.point_keeping[1];
}
//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustates how you do this relative to initializing the model.
//
int_T main(int_T argc, char **argv) {
    // Unused arguments
    (void)(argc);
    (void)(argv);

    // Initialize model
    station_keeping_Obj.initialize();

    // Attach rt_OneStep to a timer or interrupt service routine with
    //  period 0.1 seconds (the model's base sample time) here.  The
    //  call syntax for rt_OneStep is
    //

    ros::init(argc, argv, "station_keeping");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher station_keeping_pub;
    ros::Publisher station_keeping_para_pub;
    ros::Publisher mach_pub;

    station_keeping_pub = nh.advertise<sailboat_message::station_keeping_out>("station_keeping_out", 2);
    station_keeping_para_pub = nh.advertise<sailboat_message::station_keeping_para>("station_keeping_para", 2);
    mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

    dynamic_reconfigure::Server<station_keeping::station_keeping_Config> server;
    dynamic_reconfigure::Server<station_keeping::station_keeping_Config>::CallbackType f;

    f = boost::bind(&ScanningCfgcallback, _1, _2);
    server.setCallback(f);

//    dynamic_reconfigure::Server<scanning::pcCtrl_Config> server2;
//    dynamic_reconfigure::Server<scanning::pcCtrl_Config>::CallbackType f2;
//
//    f2 = boost::bind(&PcCtrlCfgcallback, _1, _2);
//    server2.setCallback(f2);



    //todo
    sub = nh.subscribe("sensor", 100, callback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rt_OneStep();
        sailboat_message::station_keeping_out msg;
        sailboat_message::Mach_msg msgMach;
        sailboat_message::station_keeping_para msgPara;

        getOutParaPut(msgPara);
        getOutMachPut(msgMach);
        getOutput(msg);

        station_keeping_pub.publish(msg);
        mach_pub.publish(msgMach);
        station_keeping_para_pub.publish(msgPara);

        loop_rate.sleep();
    }


    printf("Warning: The simulation will run forever. "
                   "Generated ERT main won't simulate model step behavior. "
                   "To change this behavior select the 'MAT-file logging' option.\n");
    fflush((NULL));
    while (rtmGetErrorStatus(station_keeping_Obj.getRTM()) == (NULL)) {
        //  Perform other application tasks here
    }

    // Disable rt_OneStep() here

    // Terminate model
    station_keeping_Obj.terminate();
    return 0;
}
//
// File trailer for generated code.
//
// [EOF]
//


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
#include <scanning/scanning.h>
#include "scanning.h"         // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/scanning_out.h"
#include "sailboat_message/scanning_para.h"
#include "sailboat_message/Mach_msg.h"

#include <dynamic_reconfigure/server.h>

#include "scanning/scanning_Config.h"


static scanningModelClass scanning_Obj;// Instance of model class
int pcCtrl = 0;
ros::Publisher scanning_para_pub;

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
void rt_OneStep(void) {
    static boolean_T OverrunFlag = 0;

    // Disable interrupts here

    // Check for overrun
    if (OverrunFlag) {
        rtmSetErrorStatus(scanning_Obj.getRTM(), "Overrun");
        return;
    }

    OverrunFlag = true;

    // Save FPU context here (if necessary)
    // Re-enable timer or interrupt here
    // Set model inputs here

    // Step the model
    scanning_Obj.step();

    // Get model outputs here

    // Indicate task complete
    OverrunFlag = false;

    // Disable interrupts here
    // Restore FPU context here (if necessary)
    // Enable interrupts here
}


void getInput(const sailboat_message::Sensor_msg::ConstPtr msg) {
    scanning_Obj.scanning_U.North = msg->Posx;
    scanning_Obj.scanning_U.East = msg->Posy;
    scanning_Obj.scanning_U.Roll = msg->Roll;
    scanning_Obj.scanning_U.Yaw = msg->Yaw;
    scanning_Obj.scanning_U.Roll_rate = msg->gx;
    scanning_Obj.scanning_U.Yaw_rate = msg->gz;
    scanning_Obj.scanning_U.Airmar_wind_angle = msg->AWA;
    scanning_Obj.scanning_U.Airmar_wind_speed = msg->AWS;
}

void ScanningCfgcallback(scanning::scanning_Config &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f", config.Kp, config.Ki, config.Kd);

    if (config.PC_Ctrl == true)
        pcCtrl = 1;
    else
        pcCtrl = 0;

    scanning_Obj.scanning_P.Kp = config.Kp;
    scanning_Obj.scanning_P.Ki = config.Ki;
    scanning_Obj.scanning_P.Kd = config.Kd;
    scanning_Obj.scanning_P.R = config.R;
    scanning_Obj.scanning_P.max_loose_time = config.max_loose_time;
    scanning_Obj.scanning_P.max_roll_allowed = config.max_roll_allowed;
    scanning_Obj.scanning_P.pos_history_len = config.pos_history_len;
    scanning_Obj.scanning_P.run_period = config.run_period;
    scanning_Obj.scanning_P.ship_speed_history_len = config.ship_speed_history_len;
    scanning_Obj.scanning_P.tacking_force_discount = config.tacking_force_discount;
    scanning_Obj.scanning_P.wind_mean_time = config.wind_mean_time;
    scanning_Obj.scanning_P.points_up_move = config.points_up_move;
    scanning_Obj.scanning_P.tacking_discount_decrease_windspeed = config.tacking_discount_decrease_windspeed;
    scanning_Obj.scanning_P.jibing_time = config.jibing_time;
    scanning_Obj.scanning_P.tacking_time = config.tacking_time;
    scanning_Obj.scanning_P.upwind_R_expand_ratio = config.upwind_R_expand_ratio;
    scanning_Obj.scanning_P.start_counting = config.start_counting;

    scanning_Obj.scanning_P.scanning_points[0] = config.point0_x;
    scanning_Obj.scanning_P.scanning_points[1] = config.point1_x;
    scanning_Obj.scanning_P.scanning_points[2] = config.point2_x;
    scanning_Obj.scanning_P.scanning_points[3] = config.point3_x;

    scanning_Obj.scanning_P.scanning_points[4] = config.point0_y;
    scanning_Obj.scanning_P.scanning_points[5] = config.point1_y;
    scanning_Obj.scanning_P.scanning_points[6] = config.point2_y;
    scanning_Obj.scanning_P.scanning_points[7] = config.point3_y;
}


void callback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    getInput(msg);
}

void getOutMachPut(sailboat_message::Mach_msg &msg) {

    msg.timestamp = ros::Time::now().toSec();
    msg.motor = 50;
    msg.rudder = scanning_Obj.scanning_Y.rudder;
    msg.sail = scanning_Obj.scanning_Y.sail;
    msg.PCCtrl = pcCtrl;

}

void getOutput(sailboat_message::scanning_out &msg) {

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.sail = scanning_Obj.scanning_Y.sail;
    msg.rudder = scanning_Obj.scanning_Y.rudder;
    msg.leg = scanning_Obj.scanning_Y.leg;
    msg.los_heading = scanning_Obj.scanning_Y.los_heading;
    msg.speed_angle = scanning_Obj.scanning_Y.speed_angle;
    msg.speed_angle_d = scanning_Obj.scanning_Y.speed_angle_d;
    msg.wind_speed = scanning_Obj.scanning_Y.wind_speed;
    msg.wind_angle_mean = scanning_Obj.scanning_Y.wind_angle_mean;
    msg.drive_force = scanning_Obj.scanning_Y.drive_force;
    msg.sail_ground_d = scanning_Obj.scanning_Y.sail_ground_d;
    msg.horizontalSpeed = scanning_Obj.scanning_Y.HorizontalSpeed;

}

void getOutParaPut(sailboat_message::scanning_para &msg) {

    msg.Kp = scanning_Obj.scanning_P.Kp;
    msg.Ki = scanning_Obj.scanning_P.Ki;
    msg.Kd = scanning_Obj.scanning_P.Kd;
    msg.R = scanning_Obj.scanning_P.R;
    msg.max_loose_time = scanning_Obj.scanning_P.max_loose_time;
    msg.max_roll_allowed = scanning_Obj.scanning_P.max_roll_allowed;
    msg.pos_history_len = scanning_Obj.scanning_P.pos_history_len;
    msg.run_period = scanning_Obj.scanning_P.run_period;
    msg.ship_speed_history_len = scanning_Obj.scanning_P.ship_speed_history_len;
    msg.tacking_force_discount = scanning_Obj.scanning_P.tacking_force_discount;
    msg.wind_mean_time = scanning_Obj.scanning_P.wind_mean_time;

    msg.points_up_move = scanning_Obj.scanning_P.points_up_move;
    msg.tacking_discount_decrease_windspeed = scanning_Obj.scanning_P.tacking_discount_decrease_windspeed;
    msg.jibing_time = scanning_Obj.scanning_P.jibing_time;
    msg.tacking_time = scanning_Obj.scanning_P.tacking_time;
    msg.upwind_R_expand_ratio = scanning_Obj.scanning_P.upwind_R_expand_ratio;
    msg.start_counting = scanning_Obj.scanning_P.start_counting;


    msg.point0_x = scanning_Obj.scanning_P.scanning_points[0];
    msg.point1_x = scanning_Obj.scanning_P.scanning_points[1];
    msg.point2_x = scanning_Obj.scanning_P.scanning_points[2];
    msg.point3_x = scanning_Obj.scanning_P.scanning_points[3];
    msg.point0_y = scanning_Obj.scanning_P.scanning_points[4];
    msg.point1_y = scanning_Obj.scanning_P.scanning_points[5];
    msg.point2_y = scanning_Obj.scanning_P.scanning_points[6];
    msg.point3_y = scanning_Obj.scanning_P.scanning_points[7];
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustates how you do this relative to initializing the model.
//
int_T main(int_T argc, char **argv) {
    // Unused arguments
    (void) (argc);
    (void) (argv);

    // Initialize model
    scanning_Obj.initialize();

    // Attach rt_OneStep to a timer or interrupt service routine with
    //  period 0.1 seconds (the model's base sample time) here.  The
    //  call syntax for rt_OneStep is
    //

    ros::init(argc, argv, "scanning");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher scanning_pub;

    ros::Publisher mach_pub;
    scanning_pub = nh.advertise<sailboat_message::scanning_out>("scanning_out", 2);
    scanning_para_pub = nh.advertise<sailboat_message::scanning_para>("scanning_para", 2);
    mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

    dynamic_reconfigure::Server<scanning::scanning_Config> server;
    dynamic_reconfigure::Server<scanning::scanning_Config>::CallbackType f;

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
        sailboat_message::scanning_out msg;
        sailboat_message::Mach_msg msgMach;
        sailboat_message::scanning_para msgPara;

        getOutMachPut(msgMach);
        getOutput(msg);
        getOutParaPut(msgPara);

        scanning_pub.publish(msg);
        scanning_para_pub.publish(msgPara);
        mach_pub.publish(msgMach);

        loop_rate.sleep();
    }


    printf("Warning: The simulation will run forever. "
                   "Generated ERT main won't simulate model step behavior. "
                   "To change this behavior select the 'MAT-file logging' option.\n");
    fflush((NULL));
//    while (rtmGetErrorStatus(scanning_Obj.getRTM()) == (NULL)) {
//        //  Perform other application tasks here
//    }

    // Disable rt_OneStep() here

    // Terminate model
    scanning_Obj.terminate();
    return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//


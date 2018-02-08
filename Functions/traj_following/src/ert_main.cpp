//
// File: ert_main.cpp
//
// Code generated for Simulink model 'path_following'.
//
// Model version                  : 1.315
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Fri Oct 20 13:07:25 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush 
#include "path_following.h"            // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/traj_following_out.h"
#include "sailboat_message/traj_following_para.h"
#include "sailboat_message/Mach_msg.h"

#include <dynamic_reconfigure/server.h>
#include <traj_following/path_following.h>

#include "traj_following/traj_following_Config.h"

static scanningModelClass path_following_Obj;// Instance of model class
int pcCtrl = 0;
ros::Publisher traj_following_para_pub;

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
        rtmSetErrorStatus(path_following_Obj.getRTM(), "Overrun");
        return;
    }

    OverrunFlag = true;

    // Save FPU context here (if necessary)
    // Re-enable timer or interrupt here
    // Set model inputs here

    // Step the model
    path_following_Obj.step();

    // Get model outputs here

    // Indicate task complete
    OverrunFlag = false;

    // Disable interrupts here
    // Restore FPU context here (if necessary)
    // Enable interrupts here
}

void getInputPath(const nav_msgs::PathConstPtr &path_msg) {
    int length = path_msg->poses.size();
    path_following_Obj.path_following_U.path_num = length;
    for (int i = 0; i < 100; i++) {
        if (i < length)
            path_following_Obj.path_following_U.path[i] = path_msg->poses[i].pose.position.x;
        else
            path_following_Obj.path_following_U.path[i] = 0;
    }
    for (int i = 0; i < 100; i++) {
        if (i < length)
            path_following_Obj.path_following_U.path[100+i] = path_msg->poses[i].pose.position.y;
        else
            path_following_Obj.path_following_U.path[100+i] = 0;
    }
}


void getInput(const sailboat_message::Sensor_msg::ConstPtr msg) {
    path_following_Obj.path_following_U.North = msg->Posx;
    path_following_Obj.path_following_U.East = msg->Posy;
    path_following_Obj.path_following_U.Roll = msg->Roll;
    path_following_Obj.path_following_U.Yaw = msg->Yaw;
    path_following_Obj.path_following_U.Roll_rate = msg->gx;
    path_following_Obj.path_following_U.Yaw_rate = msg->gz;
    path_following_Obj.path_following_U.Airmar_wind_angle = msg->AWA;
    path_following_Obj.path_following_U.Airmar_wind_speed = msg->AWS;
}

void traj_followingCfgcallback(traj_following::traj_following_Config &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f", config.Kp, config.Ki, config.Kd);

    if (config.PC_Ctrl == true)
        pcCtrl = 1;
    else
        pcCtrl = 0;

    path_following_Obj.path_following_P.Kp = config.Kp;
    path_following_Obj.path_following_P.Ki = config.Ki;
    path_following_Obj.path_following_P.Kd = config.Kd;
    path_following_Obj.path_following_P.R = config.R;
    path_following_Obj.path_following_P.jibing_time = config.jibing_time;
    path_following_Obj.path_following_P.leg = config.leg;

    path_following_Obj.path_following_P.max_loose_time = config.max_loose_time;
    path_following_Obj.path_following_P.max_roll_allowed = config.max_roll_allowed;
    path_following_Obj.path_following_P.pos_history_len = config.pos_history_len;
    path_following_Obj.path_following_P.run_period = config.run_period;
    path_following_Obj.path_following_P.ship_speed_history_len = config.ship_speed_history_len;
    path_following_Obj.path_following_P.tacking_force_discount = config.tacking_force_discount;
    path_following_Obj.path_following_P.tacking_discount_decrease_windspeed = config.tacking_discount_decrease_windspeed;
    path_following_Obj.path_following_P.tacking_time = config.tacking_time;

    path_following_Obj.path_following_P.wind_mean_time = config.wind_mean_time;
    path_following_Obj.path_following_P.upwind_R_expand_ratio = config.upwind_R_expand_ratio;


}


void callback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    getInput(msg);
}

void path_cb(const nav_msgs::PathConstPtr &path_msg) {
    getInputPath(path_msg);
}

void getOutMachPut(sailboat_message::Mach_msg &msg) {

    msg.timestamp = ros::Time::now().toSec();
    msg.motor = 0;
    msg.rudder = path_following_Obj.path_following_Y.rudder;
    msg.sail = path_following_Obj.path_following_Y.sail;
    msg.PCCtrl = pcCtrl;

}

void getOutput(sailboat_message::traj_following_out &msg) {

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.sail = path_following_Obj.path_following_Y.sail;
    msg.rudder = path_following_Obj.path_following_Y.rudder;
    msg.los_heading = path_following_Obj.path_following_Y.los_heading;
    msg.speed_angle = path_following_Obj.path_following_Y.speed_angle;
    msg.speed_angle_d = path_following_Obj.path_following_Y.speed_angle_d;
    msg.wind_speed = path_following_Obj.path_following_Y.wind_speed;
    msg.wind_angle_mean = path_following_Obj.path_following_Y.wind_angle_mean;
    msg.drive_force = path_following_Obj.path_following_Y.drive_force;
    msg.sail_ground_d = path_following_Obj.path_following_Y.sail_ground_d;
    msg.horizontalSpeed = path_following_Obj.path_following_Y.HorizontalSpeed;

}

void getOutParaPut(sailboat_message::traj_following_para &msg) {

    msg.Kp = path_following_Obj.path_following_P.Kp;
    msg.Ki = path_following_Obj.path_following_P.Ki;
    msg.Kd = path_following_Obj.path_following_P.Kd;
    msg.R = path_following_Obj.path_following_P.R;
    msg.leg = path_following_Obj.path_following_P.leg;
    msg.max_loose_time = path_following_Obj.path_following_P.max_loose_time;
    msg.max_roll_allowed = path_following_Obj.path_following_P.max_roll_allowed;
    msg.pos_history_len = path_following_Obj.path_following_P.pos_history_len;
    msg.run_period = path_following_Obj.path_following_P.run_period;
    msg.ship_speed_history_len = path_following_Obj.path_following_P.ship_speed_history_len;
    msg.tacking_force_discount = path_following_Obj.path_following_P.tacking_force_discount;
    msg.wind_mean_time = path_following_Obj.path_following_P.wind_mean_time;
    msg.tacking_discount_decrease_windspeed = path_following_Obj.path_following_P.tacking_discount_decrease_windspeed;
    msg.jibing_time = path_following_Obj.path_following_P.jibing_time;
    msg.tacking_time = path_following_Obj.path_following_P.tacking_time;
    msg.upwind_R_expand_ratio = path_following_Obj.path_following_P.upwind_R_expand_ratio;




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
    path_following_Obj.initialize();

    ros::init(argc, argv, "traj_following");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber path_sub;
    ros::Publisher traj_following_pub;
    ros::Publisher mach_pub;
    traj_following_pub = nh.advertise<sailboat_message::traj_following_out>("traj_following_out", 2);
    traj_following_para_pub = nh.advertise<sailboat_message::traj_following_para>("traj_following_para", 2);
    mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

    dynamic_reconfigure::Server<traj_following::traj_following_Config> server;
    dynamic_reconfigure::Server<traj_following::traj_following_Config>::CallbackType f;

    f = boost::bind(&traj_followingCfgcallback, _1, _2);
    server.setCallback(f);

//    dynamic_reconfigure::Server<scanning::pcCtrl_Config> server2;
//    dynamic_reconfigure::Server<scanning::pcCtrl_Config>::CallbackType f2;
//
//    f2 = boost::bind(&PcCtrlCfgcallback, _1, _2);
//    server2.setCallback(f2);



    //todo
    sub = nh.subscribe("sensor", 100, callback);
    path_sub = nh.subscribe("/planned_path", 100, path_cb);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rt_OneStep();
        sailboat_message::traj_following_out msg;
        sailboat_message::Mach_msg msgMach;
        sailboat_message::traj_following_para msgPara;

        getOutMachPut(msgMach);
        getOutput(msg);
        getOutParaPut(msgPara);

        traj_following_pub.publish(msg);
        traj_following_para_pub.publish(msgPara);
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
    while (rtmGetErrorStatus(path_following_Obj.getRTM()) == (NULL)) {
        //  Perform other application tasks here
    }

    // Disable rt_OneStep() here

    // Terminate model
    path_following_Obj.terminate();
    return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

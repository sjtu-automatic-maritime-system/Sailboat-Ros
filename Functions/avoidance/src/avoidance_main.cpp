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
#include <stdio.h>
//#include "collision_avoidance.h"                     // This ert_main.c example uses printf/fflush
#include "avoidance/collision_avoidance.h"               // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"
#include "sailboat_message/avoidance_out.h"
#include "sailboat_message/avoidance_para.h"
#include "tld_msgs/BoundingBox.h"
#include <geometry_msgs/PoseArray.h>

#include <dynamic_reconfigure/server.h>
#include <avoidance/collision_avoidance.h>
#include "avoidance/avoidance_Config.h"


static scanningModelClass collision_avoidance_Obj;
// Instance of model class
int pcCtrl = 0;

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
        rtmSetErrorStatus(collision_avoidance_Obj.getRTM(), "Overrun");
        return;
    }

    OverrunFlag = true;

    // Save FPU context here (if necessary)
    // Re-enable timer or interrupt here
    // Set model inputs here

    // Step the model
    collision_avoidance_Obj.step();

    // Get model outputs here

    // Indicate task complete
    OverrunFlag = false;

    // Disable interrupts here
    // Restore FPU context here (if necessary)
    // Enable interrupts here
}

void callback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    collision_avoidance_Obj.collision_avoidance_U.north = msg->Posx;
    collision_avoidance_Obj.collision_avoidance_U.east = msg->Posy;
    collision_avoidance_Obj.collision_avoidance_U.roll = msg->Roll;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_yaw = msg->Yaw;
    collision_avoidance_Obj.collision_avoidance_U.roll_rate = msg->gx;
    collision_avoidance_Obj.collision_avoidance_U.yaw_rate = msg->gz;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_angle = msg->AWA;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_speed = msg->AWS;
}

void cameraCallback(const tld_msgs::BoundingBox::ConstPtr msg) {
//    collision_avoidance_Obj.collision_avoidance_U.camera_confidence = msg->confidence;
}

void poseCallback(const geometry_msgs::PoseArray::ConstPtr msg) {
    int num = msg->poses.size();
    for (int i=0; i<num; i++){
        collision_avoidance_Obj.collision_avoidance_U.camera_confidence[i] = msg->poses[i].position.x;
        collision_avoidance_Obj.collision_avoidance_U.camera_confidence[100+i] = msg->poses[i].position.y;
        collision_avoidance_Obj.collision_avoidance_U.camera_confidence[200+i] = msg->poses[i].position.z;
    }
    
}

void AvoidanceCfgcallback(avoidance::avoidance_Config &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f", config.Kp, config.Ki, config.Kd);
    if (config.PC_Ctrl == true)
        pcCtrl = 1;
    else
        pcCtrl = 0;

    collision_avoidance_Obj.collision_avoidance_P.Kd                                  = config.Kd                                 ;
    collision_avoidance_Obj.collision_avoidance_P.Ki                                  = config.Ki                                 ;
    collision_avoidance_Obj.collision_avoidance_P.Kp                                  = config.Kp                                 ;
    collision_avoidance_Obj.collision_avoidance_P.R                                   = config.R                                  ;
    collision_avoidance_Obj.collision_avoidance_P.inzone_num_gate                     = config.inzone_num_gate                    ;
    collision_avoidance_Obj.collision_avoidance_P.jibing_time                         = config.jibing_time                        ;                        ;
    collision_avoidance_Obj.collision_avoidance_P.max_loose_time                      = config.max_loose_time                     ;
    collision_avoidance_Obj.collision_avoidance_P.max_roll_allowed                    = config.max_roll_allowed                   ;
    collision_avoidance_Obj.collision_avoidance_P.obs_dis_gate                        = config.obs_dis_gate                       ;
    collision_avoidance_Obj.collision_avoidance_P.obstacle_R                          = config.obstacle_R                         ;
    collision_avoidance_Obj.collision_avoidance_P.pos_history_len                     = config.pos_history_len                    ;
    collision_avoidance_Obj.collision_avoidance_P.run_period                          = config.run_period                         ;
    collision_avoidance_Obj.collision_avoidance_P.ship_speed_history_len              = config.ship_speed_history_len             ;
    collision_avoidance_Obj.collision_avoidance_P.start_counting                      = config.start_counting                     ;
    collision_avoidance_Obj.collision_avoidance_P.tacking_discount_decrease_windspeed = config.tacking_discount_decrease_windspeed;
    collision_avoidance_Obj.collision_avoidance_P.tacking_force_discount              = config.tacking_force_discount             ;
    collision_avoidance_Obj.collision_avoidance_P.tacking_speed                       = config.tacking_speed                      ;
    collision_avoidance_Obj.collision_avoidance_P.tacking_time                        = config.tacking_time                       ;
    collision_avoidance_Obj.collision_avoidance_P.upwind_R_expand_ratio               = config.upwind_R_expand_ratio              ;
    collision_avoidance_Obj.collision_avoidance_P.wind_mean_time                      = config.wind_mean_time                     ;
    collision_avoidance_Obj.collision_avoidance_P.wind_side                           = config.wind_side                          ;

    collision_avoidance_Obj.collision_avoidance_P.points[0] = config.point0_x;
    collision_avoidance_Obj.collision_avoidance_P.points[1] = config.point1_x;
    collision_avoidance_Obj.collision_avoidance_P.points[2] = config.point2_x;
    collision_avoidance_Obj.collision_avoidance_P.points[3] = config.point3_x;

    collision_avoidance_Obj.collision_avoidance_P.points[4] = config.point0_y;
    collision_avoidance_Obj.collision_avoidance_P.points[5] = config.point1_y;
    collision_avoidance_Obj.collision_avoidance_P.points[6] = config.point2_y;
    collision_avoidance_Obj.collision_avoidance_P.points[7] = config.point3_y;

}

void getOutMachPut(sailboat_message::Mach_msg &msg) {

    //msg.timestamp = ros::Time::now().toSec();
    msg.motor = 50;
    msg.rudder = collision_avoidance_Obj.collision_avoidance_Y.rudder;
    msg.sail   = collision_avoidance_Obj.collision_avoidance_Y.sail;

    msg.PCCtrl = pcCtrl;

}

void getOutput(sailboat_message::avoidance_out &msg) {

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.sail_ground_d     = collision_avoidance_Obj.collision_avoidance_Y.sail_ground_d;
    msg.max_drive_force   = collision_avoidance_Obj.collision_avoidance_Y.max_drive_force;
    msg.rudder            = collision_avoidance_Obj.collision_avoidance_Y.rudder;
    msg.sail              = collision_avoidance_Obj.collision_avoidance_Y.sail;
    msg.ship_speed        = collision_avoidance_Obj.collision_avoidance_Y.ship_speed;
    msg.speed_angle_d     = collision_avoidance_Obj.collision_avoidance_Y.speed_angle_d;
    msg.wind_angle_ground = collision_avoidance_Obj.collision_avoidance_Y.wind_angle_ground;
    msg.wind_speed        = collision_avoidance_Obj.collision_avoidance_Y.wind_speed;
    msg.leg               = collision_avoidance_Obj.collision_avoidance_Y.leg;
    msg.obstacle_judge    = collision_avoidance_Obj.collision_avoidance_Y.obstacle_judge;
}

void getOutParaPut(sailboat_message::avoidance_para &msg)
{

    msg.Kd                                  = collision_avoidance_Obj.collision_avoidance_P.Kd                                 ;
    msg.Ki                                  = collision_avoidance_Obj.collision_avoidance_P.Ki                                 ;
    msg.Kp                                  = collision_avoidance_Obj.collision_avoidance_P.Kp                                 ;
    msg.R                                   = collision_avoidance_Obj.collision_avoidance_P.R                                  ;
    msg.inzone_num_gate                     = collision_avoidance_Obj.collision_avoidance_P.inzone_num_gate                    ;
    msg.jibing_time                         = collision_avoidance_Obj.collision_avoidance_P.jibing_time                        ;                        ;
    msg.max_loose_time                      = collision_avoidance_Obj.collision_avoidance_P.max_loose_time                     ;
    msg.max_roll_allowed                    = collision_avoidance_Obj.collision_avoidance_P.max_roll_allowed                   ;
    msg.obs_dis_gate                        = collision_avoidance_Obj.collision_avoidance_P.obs_dis_gate                       ;
    msg.obstacle_R                          = collision_avoidance_Obj.collision_avoidance_P.obstacle_R                         ;
    msg.pos_history_len                     = collision_avoidance_Obj.collision_avoidance_P.pos_history_len                    ;
    msg.run_period                          = collision_avoidance_Obj.collision_avoidance_P.run_period                         ;
    msg.ship_speed_history_len              = collision_avoidance_Obj.collision_avoidance_P.ship_speed_history_len             ;
    msg.start_counting                      = collision_avoidance_Obj.collision_avoidance_P.start_counting                     ;
    msg.tacking_discount_decrease_windspeed = collision_avoidance_Obj.collision_avoidance_P.tacking_discount_decrease_windspeed;
    msg.tacking_force_discount              = collision_avoidance_Obj.collision_avoidance_P.tacking_force_discount             ;
    msg.tacking_speed                       = collision_avoidance_Obj.collision_avoidance_P.tacking_speed                      ;
    msg.tacking_time                        = collision_avoidance_Obj.collision_avoidance_P.tacking_time                       ;
    msg.upwind_R_expand_ratio               = collision_avoidance_Obj.collision_avoidance_P.upwind_R_expand_ratio              ;
    msg.wind_mean_time                      = collision_avoidance_Obj.collision_avoidance_P.wind_mean_time                     ;
    msg.wind_side                           = collision_avoidance_Obj.collision_avoidance_P.wind_side                          ;

    msg.point0_x = collision_avoidance_Obj.collision_avoidance_P.points[0];
    msg.point1_x = collision_avoidance_Obj.collision_avoidance_P.points[1];
    msg.point2_x = collision_avoidance_Obj.collision_avoidance_P.points[2];
    msg.point3_x = collision_avoidance_Obj.collision_avoidance_P.points[3];
    msg.point0_y = collision_avoidance_Obj.collision_avoidance_P.points[4];
    msg.point1_y = collision_avoidance_Obj.collision_avoidance_P.points[5];
    msg.point2_y = collision_avoidance_Obj.collision_avoidance_P.points[6];
    msg.point3_y = collision_avoidance_Obj.collision_avoidance_P.points[7];
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
    collision_avoidance_Obj.initialize();

    // Attach rt_OneStep to a timer or interrupt service routine with
    //  period 0.1 seconds (the model's base sample time) here.  The
    //  call syntax for rt_OneStep is
    //
    //   rt_OneStep();

    ros::init(argc, argv, "avoidance");
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber subCamera;
    ros::Subscriber subPose;
    ros::Publisher avoidance_pub;
    ros::Publisher avoidance_para_pub;

    ros::Publisher mach_pub;
    avoidance_pub = nh.advertise<sailboat_message::avoidance_out>("avoidance_out", 2);
    avoidance_para_pub = nh.advertise<sailboat_message::avoidance_para>("avoidance_para", 2);

    mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

    dynamic_reconfigure::Server<avoidance::avoidance_Config> server;
    dynamic_reconfigure::Server<avoidance::avoidance_Config>::CallbackType f;

    f = boost::bind(&AvoidanceCfgcallback, _1, _2);
    server.setCallback(f);

    sub = nh.subscribe("sensor_kalman_msg", 100, callback);
    subCamera = nh.subscribe("tld_tracked_object", 100, cameraCallback);
    subPose = nh.subscribe("/object/pose", 100, poseCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rt_OneStep();
        sailboat_message::avoidance_out msg;
        sailboat_message::Mach_msg msgMach;
        sailboat_message::avoidance_para msgPara;

        getOutMachPut(msgMach);
        getOutput(msg);
        getOutParaPut(msgPara);

        avoidance_pub.publish(msg);
        avoidance_para_pub.publish(msgPara);
        mach_pub.publish(msgMach);

        loop_rate.sleep();
    }



    printf("Warning: The simulation will run forever. "
                   "Generated ERT main won't simulate model step behavior. "
                   "To change this behavior select the 'MAT-file logging' option.\n");
    fflush((NULL));
    while (rtmGetErrorStatus(collision_avoidance_Obj.getRTM()) == (NULL)) {
        //  Perform other application tasks here
    }

    // Disable rt_OneStep() here

    // Terminate model
    collision_avoidance_Obj.terminate();
    return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

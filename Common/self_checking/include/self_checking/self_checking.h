#ifndef SELF_CHECKING_H
#define SELF_CHECKING_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <sailboat_message/Self_Checking_srv.h>
#include <sailboat_message/Out_Time_srv.h>
#include <sailboat_message/Out_Time_msg.h>

#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "sailboat_message/Arduino_msg.h"
#include "sailboat_message/Mach_msg.h"
//todo Steering gear service;
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseArray.h"
//#include "dynamixel_msgs/MotorStateList.h"
#include "dynamixel_workbench_msgs/JointCommand.h"

#include <iostream>
#include <pthread.h>
#include <cmath>

#include <stdio.h>
#include <sys/statfs.h>
#include <sys/vfs.h>
#include <errno.h>



class SailboatSelfChecking{
public:
    // SailboatSelfChecking();
    
    // ~SailboatSelfChecking();

    void onInit();

    void AHRSSubscriberCB(const sailboat_message::Ahrs_msg::ConstPtr &msg);

    void WTSTSubscriberCB(const sailboat_message::WTST_msg::ConstPtr &msg);

    void ArduinoSubscriberCB(const sailboat_message::Arduino_msg::ConstPtr &msg);

    //void DynamixelSubscriberCB(const dynamixel_msgs::MotorStateList::ConstPtr &msg);

    void CameraSubscribeCB(const sensor_msgs::Image::ConstPtr &msg);

    void RadarSubscribeCB(const geometry_msgs::PoseArray::ConstPtr &msg);

    void MachSubscribeCB(const sailboat_message::Mach_msg::ConstPtr &msg);

    void checkAHRS();

    void checkWTST();

    void checkArduino();

    void checkDynamixel();

    void checkCamera();

    void checkRader();

    void checkDisk();

    void sendresult();

    void stateUpdate();

    void onSelfCheckingInit();
    static void* threadSelfChecking(void* args);

    void onOutTimeInit();
    static void* threadOutTime(void* args);

    static SailboatSelfChecking& getInstance();

private:
    ros::NodeHandle nh;

    ros::Subscriber AHRSSubscriber;
    ros::Subscriber WTSTSubscriber;
    ros::Subscriber ArduinoSubscriber;
    ros::Subscriber CameraSubscriber;
    ros::Subscriber RadarSubscriber;
    //ros::Subscriber DynamixelSubscriber;
    ros::Subscriber MachSubscriber;

    ros::ServiceClient DynamixelCtlClient;
    ros::ServiceClient CheckResultClient;
    ros::ServiceClient OutTimeClient;

    ros::Publisher OutTimePub;

    //ros::ServiceServer SelfCheckingService;
    //ros::ServiceServer OutTimeService;

    bool init_checking = false;
    bool finish_checking = false;

    int timeout;

    float checkAHRS_param;
    float checkWTST_param;
    float checkArduino_param;
    float checkCamera_param;
    float checkRadar_param;
    float checkDynamixel_param;
    float checkDisk_param;
    
    int checkAHRS_result; // 0 init false 1 init normal
    int checkWTST_result;
    int checkArduino_result;
    int checkCamera_result;
    int checkRadar_result;
    int checkDynamixel_result;
    int checkDisk_result;

    bool AHRS_outTime = false;
    bool WTST_outTime = false;
    bool Arduino_outTime = false;
    bool Mach_outTime = false;

    int ahrsMsgNum = 0;
    ros::Time ahrs_timestamp;
    ros::Time ahrs_timestamp_last;
    double ahrs_time_sum = 0;
    double ahrs_hz = 0;

    int wtstMsgNum = 0;
    ros::Time wtst_timestamp;
    ros::Time wtst_timestamp_last;
    double wtst_time_sum = 0;
    double wtst_hz = 0;

    int arduinoMsgNum = 0;
    ros::Time arduino_timestamp;
    ros::Time arduino_timestamp_last;
    double arduino_time_sum = 0;
    double arduino_hz = 0;
    
    int cameraMsgNum = 0;
    ros::Time camera_timestamp;
    ros::Time camera_timestamp_last;
    double camera_time_sum = 0;
    double camera_hz = 0;

    int radarMsgNum = 0;
    ros::Time radar_timestamp;
    ros::Time radar_timestamp_last;
    double radar_time_sum = 0;
    double radar_hz = 0;

    ros::Time mach_timestamp;
    ros::Time mach_timestamp_last;

    int dynamixel_sail = 0;
    int dynamixel_rudder = 0;

};


#endif
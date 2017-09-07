//
// Created by hywel on 17-7-6.
//

//#include "Sensor_filters.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sailboat_message/Ahrs_msg.h"
#include "sailboat_message/WTST_msg.h"
#include "sailboat_message/Sensor_msg.h"

using namespace message_filters;

static double SensorMsg[10];

void callback(const sailboat_message::Ahrs_msg::ConstPtr& msg1, const sailboat_message::WTST_msg::ConstPtr& msg2)
{
    //ROSINFO('time :[%f] [%f]',msg1->header.stamp,msg2->header.stamp);
    SensorMsg[0] = 0; //todo!
    SensorMsg[1] = 0; //todo!
    SensorMsg[2] = msg1->gx;
    SensorMsg[3] = msg1->gz;
    SensorMsg[4] = msg2->PosX;
    SensorMsg[5] = msg2->PosY;
    SensorMsg[6] = msg2->Roll/57.3;
    SensorMsg[7] = msg2->Yaw/57.3;
    SensorMsg[8] = msg2->WindAngle/57.3;
    SensorMsg[9] = msg2->WindSpeed*0.514;
    if (SensorMsg[8]>3.1415926)
        SensorMsg[8] = SensorMsg[8]-3.1415926*2;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_filters");

    ros::NodeHandle nh;
    ros::Publisher sensor_pub;

    sensor_pub = nh.advertise<sailboat_message::Sensor_msg>("sensor", 2);
    message_filters::Subscriber<sailboat_message::Ahrs_msg> ahrs_sub(nh, "ahrs", 1);
    message_filters::Subscriber<sailboat_message::WTST_msg> wtst_sub(nh, "wtst", 1);

    typedef sync_policies::ApproximateTime<sailboat_message::Ahrs_msg, sailboat_message::WTST_msg> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ahrs_sub, wtst_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(10);



    while (ros::ok())
    {

        sailboat_message::Sensor_msg msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "sensor";
        msg.ux =   SensorMsg[0];
        msg.vy =   SensorMsg[1];
        msg.gx =   SensorMsg[2];
        msg.gz =   SensorMsg[3];
        msg.Posx = SensorMsg[4];
        msg.Posy = SensorMsg[5];
        msg.Roll = SensorMsg[6];
        msg.Yaw =  SensorMsg[7];
        msg.AWA =  SensorMsg[8];
        msg.AWS =  SensorMsg[9];

        sensor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
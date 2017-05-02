//
// Created by hywel on 17-4-23.
//

#include "CHeadingControl.h"
#include "sailboat_actuator/pid_adjustment_Config.h"

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "HeadingControl");

    //Create an object of class SubscribeAndPublish that will take care of everything
    CHeadingControl headingControl(2,0,0);

    dynamic_reconfigure::Server<sailboat_actuator::pid_adjustment_Config> srv;
    dynamic_reconfigure::Server<sailboat_actuator::pid_adjustment_Config>::CallbackType f;

    f = boost::bind(&CHeadingControl::PIDCallback, &headingControl, _1, _2);
    srv.setCallback(f);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        headingControl.AP_Calc(); //计算
        double rudder = headingControl.Get_Rudder();
        double sail = headingControl.Get_Sail();

        sailboat_message::Mach_msg msg;
        msg.timestamp = ros::Time::now().toSec();
        msg.motor = 0;
        msg.rudder = rudder;
        msg.sail = sail;

        ROS_INFO("I talk Rudder_Angle: [%f]", msg.rudder);
        headingControl.mach_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
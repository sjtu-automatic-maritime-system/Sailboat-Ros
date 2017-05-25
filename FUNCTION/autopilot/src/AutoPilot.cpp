//
// Created by hywel on 17-4-23.
//

#include "autopilot_lib/CAutopilotVer1.h"
#include "autopilot_lib/pid_adjustment_Config.h"





int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "autopilot_ver1");

    //Create an object of class SubscribeAndPublish that will take care of everything
    CAutopilotVer1 autopilot(0.5,0,0,0.1,0.52,-0.52);

    dynamic_reconfigure::Server<autopilot_lib::pid_adjustment_Config> srv;
    dynamic_reconfigure::Server<autopilot_lib::pid_adjustment_Config>::CallbackType f;

    f = boost::bind(&CAutopilotVer1::PIDCallback, &autopilot, _1, _2);
    srv.setCallback(f);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        autopilot.AP_Calc(); //计算
        double rudder = autopilot.Get_Rudder();
        double sail = autopilot.Get_Sail();
        int PCCtrl = autopilot.Get_PCCtrl();

        mach_onboat::Mach_msg msg;
        msg.timestamp = ros::Time::now().toSec();
        msg.motor = 50;
        msg.rudder = rudder;
        msg.sail = sail;
        msg.PCCtrl = PCCtrl;

        //ROS_INFO("I talk Rudder_Angle: [%f]", msg.rudder);
        //ROS_INFO("I talk Sail_Angle: [%f]", msg.sail);
        autopilot.mach_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

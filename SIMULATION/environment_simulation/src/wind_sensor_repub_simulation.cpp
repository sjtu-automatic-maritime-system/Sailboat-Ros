//
// Created by hywel on 17-3-23.
//

#include "ros/ros.h"
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Wind_Simulation_msg.h"
#include "sailboat_message/WTST_Pro_msg.h"


double true_wind = 0;
double posx = 0;
double posy = 0;
double heading = 0;

void wind_simulation_cb(const sailboat_message::Wind_Simulation_msgConstPtr &wind_sim_in) {
    true_wind = wind_sim_in->TWA;
}

void sensor_cb(const sailboat_message::Sensor_msgConstPtr &sensor_in) {
    posx = sensor_in->Posx;
    posy = sensor_in->Posy;
    heading = sensor_in->Yaw;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "wind_sensor_repub");
    ros::NodeHandle nh;

    ros::Publisher wtst_pro_pub = nh.advertise<sailboat_message::WTST_Pro_msg>("/wtst_pro", 2);

    ros::Subscriber sensor_sub = nh.subscribe("/sensor", 2, &sensor_cb);
    ros::Subscriber wind_sub = nh.subscribe("/wind", 2, &wind_simulation_cb);


    ros::Rate loop_rate(10);

    while (ros::ok()) {
        sailboat_message::WTST_Pro_msg wtst_pro_msg;

        wtst_pro_msg.PosX = posx;
        wtst_pro_msg.PosY = posy;
        wtst_pro_msg.Yaw = heading * 57.3;
        wtst_pro_msg.WindDirectionTrue = true_wind * 57.3;

        wtst_pro_pub.publish(wtst_pro_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

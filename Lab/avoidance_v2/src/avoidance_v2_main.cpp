#include <iostream>
#include <stddef.h>
#include <stdio.h>
#include "avoidance_v2/avoidance_v2.h"
#include "ros/ros.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Target_msg.h"
#include "sailboat_message/obs_msg.h"
#include "sailboat_message/obs_out.h"

#include "geometry_msgs/PoseArray.h"


//Create an object of class scanningModelClass that will take care of everything
scanningModelClass collision_avoidance_Obj;


void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    ROS_INFO("[Message] Sensor_msg sub: [AWA is %f and AWS is %f]", msg->AWA, msg->AWS);
    collision_avoidance_Obj.collision_avoidance_U.north = msg->Posx;
    collision_avoidance_Obj.collision_avoidance_U.east = msg->Posy;
    collision_avoidance_Obj.collision_avoidance_U.roll = msg->Roll;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_yaw = msg->Yaw;
    collision_avoidance_Obj.collision_avoidance_U.roll_rate = msg->gx;
    collision_avoidance_Obj.collision_avoidance_U.yaw_rate = msg->gz;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_angle = msg->AWA;
    collision_avoidance_Obj.collision_avoidance_U.Airmar_wind_speed = msg->AWS;
}



void obstacleCallback(const sailboat_message::obs_msg::ConstPtr& msg){
    ROS_INFO("[Message] Obstacle_msg sub: [%f %f %f %f %f %f %f %f] ", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
    collision_avoidance_Obj.obstacle_information.data.clear();
    collision_avoidance_Obj.obstacle_information.data.assign(msg->data.begin(),msg->data.end());

    collision_avoidance_Obj.target_information.angle = msg->target_angle;

    /*//another way to copy one vector to another
    for (std::vector<double> ::iterator iter= msg->data.begin();iter!=msg->data.end();iter++){
        collision_avoidance_Obj.obstacle_information.data.push_back(*iter);
    }
    */
}

void detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg){

    if (msg->poses.size() == 0 ){
        ROS_INFO("[Message] No detection information!");
        return;
    }

    collision_avoidance_Obj.obstacle_pose.x = msg->poses[0].position.x;
    collision_avoidance_Obj.obstacle_pose.y = msg->poses[0].position.y;

}

void staticWindCallback(const sailboat_message::obs_msg::ConstPtr& msg){
    ROS_INFO("[Message] static_wind_msg sub: [wind angle is %f and wind speed is %f]", msg->wind_angle, msg->wind_speed);
    collision_avoidance_Obj.static_wind_information.wind_angle = msg->wind_angle;
    collision_avoidance_Obj.static_wind_information.wind_speed = msg->wind_speed;
}

/*//public version of target_msg
void targetCallback(const sailboat_message::Target_msg::ConstPtr &msg) {
    ROS_INFO("[Message] Target_msg sub: [%f]",msg->TargetAngle);
    collision_avoidance_Obj.target_information.angle = msg->TargetAngle;
}
*/

/*
//** for test ** private version of target_msg
void targetCallback(const sailboat_message::obs_msg::ConstPtr &msg) {
    ROS_INFO("[Message] Target_msg sub: [%f]",msg->target_angle);
    collision_avoidance_Obj.target_information.angle = msg->target_angle;
}

*/

void getOutput(sailboat_message::Target_msg &msg) {

    //msg.timestamp = ros::Time::now().toSec();
    msg.TargetAngle = collision_avoidance_Obj.output.angle;
    ROS_INFO("[Publish] I published: the output angle is %f", msg.TargetAngle);
}


int_T main(int_T argc, char **argv)
{
    // Unused arguments
    (void)(argc);
    (void)(argv);

    // Initialize model
    collision_avoidance_Obj.initialize();

    ros::init(argc, argv, "avoidance");
    ros::NodeHandle avoidance_node;

    ros::Publisher avoidance_pub;
    avoidance_pub = avoidance_node.advertise<sailboat_message::Target_msg>("targetangle", 2);
    
    ros::Subscriber avoidance_sub_sensor = avoidance_node.subscribe("sensor", 2, sensorCallback);

    ros::Subscriber avoidance_sub_obs = avoidance_node.subscribe("detection_msg", 2, obstacleCallback);

    ros::Subscriber avoidance_sub_detection = avoidance_node.subscribe("/object/pose", 2, detectionCallback);

    //ros::Subscriber avoidance_sub_wind;
    //avoidance_sub_wind = avoidance_node.subscribe("obs_msg", 100, staticWindCallback);

    //ros::Subscriber avoidance_sub_tar;
    //avoidance_sub_tar = avoidance_node.subscribe("obs_msg", 2, targetCallback);

    ros::Rate loop_rate(10);

    
    while (ros::ok()) {
        collision_avoidance_Obj.avoidance_algo();
        //std::cout << collision_avoidance_Obj.obstacle_information.data.size();

        sailboat_message::Target_msg pub_msg;

        getOutput(pub_msg);

        avoidance_pub.publish(pub_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    

    return 0;
}
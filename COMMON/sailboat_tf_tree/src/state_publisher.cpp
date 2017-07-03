//
// Created by hywel on 17-5-14.
//

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "sailboat_message/Sensor_msg.h"
#include "mach_onboat/Mach_msg.h"

double ux;
double vy;
double wx;
double wz;
double Posx = 0.0;
double Posy = 0.0;
double Roll = 0.0;
double Yaw = 0.0;

double rudder = 0.0;
double sail = 0.0;


void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr& msg) {
    ux = msg->ux;
    vy = msg->vy;
    wx = msg->gx;
    wz = msg->gz;
    Posx = msg->Posx;
    Posy = msg->Posy;
    Roll = msg->Roll;
    Yaw = msg->Yaw;
    //ROS_INFO("I heard: [%f] [%f] [%f] [%f]", msg->Posx,msg->Posy,Posx,Posy);
}

void machCallback(const mach_onboat::Mach_msg::ConstPtr& msg) {
    ROS_INFO("I heard: [%f] [%f]", msg->rudder,msg->sail);
    rudder = msg->rudder;
    sail = msg->sail;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Subscriber sensor_sub = n.subscribe("sensor", 2, &sensorCallback);
    ros::Subscriber mach_sub = n.subscribe("mach", 2, &machCallback);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);

    const double degree = M_PI/180;

    // robot state
    double boat_to_rudder = 0;
    double boat_to_sail=0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    nav_msgs::Odometry odometry;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //ROS_INFO("I heard: [%f] [%f]", Posx,Posy);
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="boat_to_sail";
        joint_state.position[0] = sail;
        joint_state.name[1] ="boat_to_rudder";
        joint_state.position[1] = rudder*2;
        //ROS_INFO("I talk: [%f] [%f] [%f]", degree,sail,rudder);

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        //odom_trans.transform.setOrigin( tf::Vector3(Posx,Posy, 0.0) );
        //tf::Quaternion q;
        //q.setRPY(Roll, 0, Yaw);
        odom_trans.transform.translation.x = Posx;
        odom_trans.transform.translation.y = Posy;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(Roll, 0, Yaw);


        odometry.header.stamp = ros::Time::now();
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_link";
        odometry.pose.pose.position.x = Posx;
        odometry.pose.pose.position.y = Posy;
        odometry.pose.pose.position.z = 0;

        odometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Roll, 0, Yaw);

        odometry.twist.twist.linear.x = ux;
        odometry.twist.twist.linear.y = vy;
        odometry.twist.twist.linear.z = 0;
        odometry.twist.twist.angular.x = wx;
        odometry.twist.twist.angular.y = 0;
        odometry.twist.twist.angular.z = wz;




        //send the joint state and transform
        joint_pub.publish(joint_state);
        odom_pub.publish(odometry);
        broadcaster.sendTransform(odom_trans);


        ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
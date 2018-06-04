//by Boxian Deng, Jun 03, 2018
//main program for obstacle avoidance
#include "avoidance_v2.h"
//messages
#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Target_msg.h"
#include "sailboat_message/obs_msg.h"

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
    if (msg->data.empty()){
        ROS_INFO("[Message] No obstacle information!");
        return;
    }

    collision_avoidance_Obj.obstacle_information.data.clear();
    collision_avoidance_Obj.obstacle_information.data.assign(msg->data.begin(),msg->data.end());
    collision_avoidance_Obj.obstacle_information.obstacle_dist = msg->obs_distance;

    collision_avoidance_Obj.target_information.angle = msg->target_angle;

    /*//another way to copy one vector to another
    for (std::vector<double> ::iterator iter= msg->data.begin();iter!=msg->data.end();iter++){
        collision_avoidance_Obj.obstacle_information.data.push_back(*iter);
    }
    */
}

/*
void staticWindCallback(const sailboat_message::obs_msg::ConstPtr& msg){
    ROS_INFO("[Message] static_wind_msg sub: [wind angle is %f and wind speed is %f]", msg->wind_angle, msg->wind_speed);
    collision_avoidance_Obj.static_wind_information.wind_angle = msg->wind_angle;
    collision_avoidance_Obj.static_wind_information.wind_speed = msg->wind_speed;
}
*/

void getOutput(sailboat_message::Target_msg &msg) {
    //msg.timestamp = ros::Time::now().toSec();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.TargetAngle = collision_avoidance_Obj.output.angle;
    ROS_INFO("[Publish] I published: the output angle is %f", msg.TargetAngle);
}


int_T main(int_T argc, char **argv){
    // Unused arguments
    (void)(argc);
    (void)(argv);

    // Initialize model
    collision_avoidance_Obj.initialize();
    collision_avoidance_Obj.initialize_vpp();

    ros::init(argc, argv, "avoidance");
    ros::NodeHandle avoidance_node;

    ros::Publisher avoidance_pub = avoidance_node.advertise<sailboat_message::Target_msg>("targetangle", 2);
    
    ros::Subscriber avoidance_sub_sensor = avoidance_node.subscribe("sensor", 2, sensorCallback);
    ros::Subscriber avoidance_sub_obs = avoidance_node.subscribe("detection_msg", 2, obstacleCallback);

    //ros::Subscriber avoidance_sub_wind;
    //avoidance_sub_wind = avoidance_node.subscribe("obs_msg", 100, staticWindCallback);

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
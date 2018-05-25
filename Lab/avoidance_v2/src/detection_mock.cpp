#include "ros/ros.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include "math.h"

#include "avoidance_v2/avoidance_v2.h"

#include "sailboat_message/obs_msg.h"
#include "sailboat_message/Sensor_msg.h"

#include "geometry_msgs/PoseArray.h"

#include <visualization_msgs/Marker.h>

double obs_pos_x;
double obs_pos_y;
double obs_radius = 5.0;

static double tar_pos_x = 60.0;
static double tar_pos_y = 0.0;

static double obj_pos_x;
static double obj_pos_y;

int flag = 0;

void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr msg)
{
  obj_pos_x = msg->Posx;
  obj_pos_y = msg->Posy;
}

void detectionCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{

  if (msg->poses.size() == 0)
  {
    ROS_INFO("[Message] No detection information!");
    return;
  }

  flag = 1;

  obs_pos_x = msg->poses[0].position.x;
  obs_pos_y = msg->poses[0].position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_pub_pose");

  obj_pos_x = 0.0;
  obj_pos_y = 0.0;

  ros::NodeHandle n;

  ros::Subscriber detection_sub_boat = n.subscribe("sensor", 2, sensorCallback);

  ros::Subscriber detection_sub_detection = n.subscribe("/object/pose", 2, detectionCallback);
  ros::Publisher detection_pub = n.advertise<sailboat_message::obs_msg>("detection_msg", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  while (true)
  {
    if (flag == 0)
    {
      sleep(0.5);
    }
    else
    {
      break;
    }
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sailboat_message::obs_msg msg;

    ROS_INFO("I got sensor message, %f, %f", obj_pos_x, obj_pos_y);

    //R is distance from sailboat to the center of obstacle
    double R = sqrt((obs_pos_x - obj_pos_x) * (obs_pos_x - obj_pos_x) + (obs_pos_y - obj_pos_y) * (obs_pos_y - obj_pos_y));

    //r is radius of circle obstacle
    double r = obs_radius;

    std::vector<double> i_angle;

    std::vector<double> obs_distance;

    //obstacle_angle is the relative angle of obastacle center to sailboat
    double obstacle_angle;

    if (obs_pos_x < obj_pos_x)
    {

      for (int i = 0; i < ANGLE_DENSITY; ++i)
      {
        i_angle.push_back(i * (2 * PI / ANGLE_DENSITY));
      }

      obstacle_angle = atan((obs_pos_y - obj_pos_y) / (obs_pos_x - obj_pos_x)) + PI;
    }

    else
    {

      for (int i = 0; i < ANGLE_DENSITY / 2; ++i)
      {
        i_angle.push_back(i * (2 * PI / ANGLE_DENSITY));
      }

      for (int i = ANGLE_DENSITY / 2; i < ANGLE_DENSITY; ++i)
      {
        i_angle.push_back(i * (2 * PI / ANGLE_DENSITY) - 2 * PI);
      }

      obstacle_angle = atan((obs_pos_y - obj_pos_y) / (obs_pos_x - obj_pos_x));
    }

    //std::cout << "obstacle_angle is "<< obstacle_angle << std::endl;  //for debugging
    for (std::vector<double>::iterator iter = i_angle.begin(); iter != i_angle.end(); iter++)
    {
      if (*iter < (obstacle_angle - asin(r / R)) || *iter > (obstacle_angle + asin(r / R)))
      {
        obs_distance.push_back(INFINITY_DOUBLE);
      }

      else
      {
        double theta = fabs(*iter - obstacle_angle);
        //std::cout << *iter * 180 / PI<< "'s theta is " << theta * 180 / PI << std::endl;
        double dist = R * cos(theta) - sqrt(r * r - R * R * sin(theta) * sin(theta));
        obs_distance.push_back(dist);
      }
    }

    msg.data.assign(obs_distance.begin(), obs_distance.end());

    //get target angle
    double targetAngle;
    if (tar_pos_x < obj_pos_x)
    {
      targetAngle = atan((tar_pos_y - obj_pos_y) / (tar_pos_x - obj_pos_x)) + PI;
    }
    else
    {
      targetAngle = atan((tar_pos_y - obj_pos_y) / (tar_pos_x - obj_pos_x));
    }

    msg.target_angle = targetAngle;

    //ROS_INFO("[Publisher from detection_mock] Obstacle information: %f %f %f %f %f %f %f %f", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    detection_pub.publish(msg);

    //rviz simulation
    visualization_msgs::Marker obs_points;

    obs_points.header.frame_id = "/odom";
    obs_points.header.stamp = ros::Time::now();

    obs_points.id = 0;
    obs_points.type = visualization_msgs::Marker::SPHERE_LIST;

    // obs_POINTS markers use x and y scale for width/height respectively
    obs_points.scale.x = obs_radius * 2;
    obs_points.scale.y = obs_radius * 2;

    // obs_Points are brown
    obs_points.color.r = 0.8;
    obs_points.color.g = 0.4;
    obs_points.color.b = 0.11;
    obs_points.color.a = 1.0;

    geometry_msgs::Point p_obs;
    p_obs.x = obs_pos_x;
    p_obs.y = obs_pos_y;
    obs_points.points.push_back(p_obs);

    marker_pub.publish(obs_points);

    //rviz simulation
    visualization_msgs::Marker tar_points;

    tar_points.header.frame_id = "/odom";
    tar_points.header.stamp = ros::Time::now();

    tar_points.id = 1;
    tar_points.type = visualization_msgs::Marker::SPHERE_LIST;

    // tar_POINTS markers use x and y scale for width/height respectively
    tar_points.scale.x = 1.0;
    tar_points.scale.y = 1.0;

    // tar_Points are brown
    tar_points.color.r = 1.0;
    tar_points.color.a = 1.0;

    geometry_msgs::Point p_tar;
    p_tar.x = tar_pos_x;
    p_tar.y = tar_pos_y;
    tar_points.points.push_back(p_tar);

    marker_pub.publish(tar_points);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

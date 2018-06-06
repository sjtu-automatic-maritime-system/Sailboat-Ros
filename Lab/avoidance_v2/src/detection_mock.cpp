//by Boxian Deng, Jun 03, 2018
//deal with obstacle detection information publisher
//used for circle obstacles
#include "avoidance_v2.h"
//messages
#include "geometry_msgs/PoseArray.h"
#include "sailboat_message/obs_msg.h"
#include "sailboat_message/Sensor_msg.h"
//rviz visualization
#include "visualization_msgs/Marker.h"


std::vector<double> obs_pos_x;
std::vector<double> obs_pos_y;
double obs_radius = 5.0;

static double tar_pos_x = 100.0;
static double tar_pos_y = 0.0;

static double obj_pos_x;
static double obj_pos_y;

static int flag=0;

void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr msg) {
    obj_pos_x = msg->Posx;
    obj_pos_y = msg->Posy;
}


void detectionCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    if (msg->poses.size() == 0 ){
        ROS_INFO("[Message] No detection information!");
        return;
    }
    obs_pos_x.clear();
    obs_pos_y.clear();
    for (int i = 0; i < msg->poses.size(); i++){
      obs_pos_x.push_back(msg->poses[i].position.x);
      obs_pos_y.push_back(msg->poses[i].position.y);
    }
    flag = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection");
  ros::NodeHandle n;

  ros::Subscriber detection_sub_boat = n.subscribe("sensor", 2, sensorCallback);
  ros::Subscriber detection_sub_detection = n.subscribe("obstaclePosition", 2, detectionCallback);
  //ros::Subscriber detection_sub_detection = n.subscribe("/object/pose", 2, detectionCallback);

  ros::Publisher detection_pub = n.advertise<sailboat_message::obs_msg>("detection_msg", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10); 

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sailboat_message::obs_msg msg;

    //if haven't received obstacle messages yet, add an obstacle far far away
    if (flag == 0){
      obs_pos_x.clear();
      obs_pos_y.clear();
      obs_pos_x.push_back(INFINITY_DOUBLE);
      obs_pos_y.push_back(INFINITY_DOUBLE);
    }

    std::vector<double> min_distances; // a double vector containing the minimum distances(considering all the obstalces) of all directions
    std::vector<double> dist_boat_obs; // a double vector containing distances from the sailboat to each obstacle

    for (int i=0; i < ANGLE_DENSITY; ++i){
      min_distances.push_back(5*INFINITY_DOUBLE);
    }
    dist_boat_obs.push_back(5*INFINITY_DOUBLE);

    int obs_num = obs_pos_x.size();
    for (int k=0; k < obs_num; ++k){
      double curr_obs_x = obs_pos_x[k];
      double curr_obs_y = obs_pos_y[k];

      //R is distance from sailboat to the center of obstacle
      double R = sqrt((curr_obs_x - obj_pos_x)*(curr_obs_x - obj_pos_x) + (curr_obs_y - obj_pos_y)*(curr_obs_y - obj_pos_y)); 

      //r is radius of circle obstacle
      double r = obs_radius;

      std::vector<double> i_angle;

      std::vector<double> curr_distance;

      //obstacle_angle is the relative angle of obastacle center to sailboat
      double obstacle_angle;

      if (curr_obs_x < obj_pos_x){

        for (int i = 0; i < ANGLE_DENSITY; ++i){
          i_angle.push_back(i * (2 * PI / ANGLE_DENSITY));
        }

        obstacle_angle  = atan((curr_obs_y - obj_pos_y) / (curr_obs_x - obj_pos_x)) + PI;

      }

      else{

        for (int i = 0; i < ANGLE_DENSITY / 2; ++i){
          i_angle.push_back(i * (2 * PI / ANGLE_DENSITY));
        }

        for (int i = ANGLE_DENSITY / 2; i < ANGLE_DENSITY; ++i){
          i_angle.push_back(i * (2 * PI / ANGLE_DENSITY) - 2 * PI);
        }
        obstacle_angle  = atan((curr_obs_y - obj_pos_y) / (curr_obs_x - obj_pos_x));
      }

      //std::cout << "obstacle_angle is "<< obstacle_angle << std::endl;  //for debugging
      for (std::vector<double> ::iterator iter= i_angle.begin();iter!=i_angle.end();iter++){
        if (*iter < (obstacle_angle - asin(r/R)) || *iter > (obstacle_angle + asin(r/R)) ){
          curr_distance.push_back(INFINITY_DOUBLE);
        }

        else {
          double theta = fabs(*iter - obstacle_angle);
          //std::cout << *iter * 180 / PI<< "'s theta is " << theta * 180 / PI << std::endl;
          double dist = R * cos(theta) - sqrt(r * r - R * R * sin(theta) * sin(theta));
          curr_distance.push_back(dist);
        }
      }

      for (int j = 0; j < ANGLE_DENSITY; ++j){
        if (curr_distance[j] < min_distances[j]){
          min_distances[j] = curr_distance[j];
        }
      }
      dist_boat_obs.push_back(R);
    }

    //copy the data of minimum distances of all directions to message and publish
    msg.data.assign(min_distances.begin(), min_distances.end());

    //get target point angle and publish
    double targetAngle;
    if (tar_pos_x < obj_pos_x){
      targetAngle = atan((tar_pos_y - obj_pos_y) / (tar_pos_x - obj_pos_x)) + PI;
    }
    else {
      targetAngle = atan((tar_pos_y - obj_pos_y) / (tar_pos_x - obj_pos_x));
    }

    msg.target_angle = targetAngle;

    //choose the nearest obstacle; find it and publish its distance and id
    double dist_nearest_obs = 5*INFINITY_DOUBLE;
    int counter = 0;
    for (int i = 0; i < dist_boat_obs.size(); ++i){
      if (dist_boat_obs[i] < dist_nearest_obs){
        dist_nearest_obs = dist_boat_obs[i];
        counter = i;
      }
    }
    msg.obs_distance = dist_nearest_obs;
    msg.nearest_obs_id = counter;

    detection_pub.publish(msg);

    //rviz visualizationfor target point
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

    //rviz visualization for obstacles
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

    ROS_INFO("[Detection] I detected %d obstacles", obs_num);
    if (obs_num > 0){
      for (int i = 0; i < obs_num; ++i){
        geometry_msgs::Point p_obs;
        p_obs.x = obs_pos_x[i];
        p_obs.y = obs_pos_y[i];
        obs_points.points.push_back(p_obs);
      }
    }

    marker_pub.publish(obs_points); 

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

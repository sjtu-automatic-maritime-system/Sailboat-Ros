//by Boxian Deng, Jun 03, 2018
//header for obstacle avoidance
#ifndef SAILBOAT_OBSTACLE_AVOIDANCE_H
#define SAILBOAT_OBSTACLE_AVOIDANCE_H
//ros header
#include "ros/ros.h"
//data structure
#include "rtwtypes.h"
//basic libraries
#include <cmath>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <vector>
#include <iostream>
#include <fstream>  
#include <sstream>  
#include <stdio.h>
#endif

//definations for useful constants
#define INFINITY_DOUBLE 1000000000.0
#define PI 3.1415926

//definations of parameters for avoidance algorithm
#define MAX_DISTANCE 30 //beyond this distance to ignore obstacles
#define MIN_DISTANCE 10 //within this distance to absolutely avoid
#define NO_TACKING_DISTANCE 5 //distance for no-tacking strategy near obstacles
#define ANGLE_DENSITY 72  //discretized 360 degrees into how many angles

typedef struct {
  real_T roll_rate;                    // '<Root>/roll_rate'
  real_T camera_confidence;            // '<Root>/camera_confidence'
  real_T north;                        // '<Root>/north'
  real_T east;                         // '<Root>/east'
  real_T Airmar_yaw;                   // '<Root>/Airmar_yaw'
  real_T roll;                         // '<Root>/roll'
  real_T Airmar_wind_speed;            // '<Root>/Airmar_wind_speed'
  real_T Airmar_wind_angle;            // '<Root>/Airmar_wind_angle'
  real_T yaw_rate;                     // '<Root>/yaw_rate'
} ExtU_collision_avoidance_T;


typedef struct {
  std::vector<double> data;             //In message "obs_msg.msg", float64[] is a data structure of vector<double>
  double obstacle_dist;
} Obstacle_information_T;

typedef struct{
  double wind_angle;
  double wind_speed;
} Static_wind_information_T;

typedef struct {
  double angle;
} Target_information_T;

typedef struct{
  double angle;
} Obstacle_output_T;



class scanningModelClass {
  // public data and function members
 public:

  // External inputs
  ExtU_collision_avoidance_T collision_avoidance_U;

  //obstacle information of minimum distance from obstacle to the sailboat of all directions
  Obstacle_information_T obstacle_information;

  //Static_wind_information_T static_wind_information;//not used

  Target_information_T target_information;

  //output angle of the algorithm
  Obstacle_output_T output;

  //VPP results of the sailboat
  double **vpps;

  //Constructor
  scanningModelClass(){};

  //Destructor
  ~scanningModelClass(){
    //Destruct VPP
    for (int i = 0; i < 121; i++){
      delete [] vpps[i];
    }
    delete vpps;  
  };

  //initialization
  void initialize();
  //read vpp data file
  void initialize_vpp();

  void avoidance_algo();

  // private data and function members
  private:

};



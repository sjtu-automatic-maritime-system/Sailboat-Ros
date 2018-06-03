#include <cmath>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include <vector>

//definations for useful constants
#define INFINITY_DOUBLE 1000000000.0
#define PI 3.1415926

//definations of parameters for avoidance algorithm
#define MAX_DISTANCE 35
#define MIN_DISTANCE 15
#define ANGLE_DENSITY 30
#define SPAN_OF_NOGO_ZONE (30.0 * PI / 180)




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
  real_T sail_ground_d;                // '<Root>/sail_ground_d'
  real_T max_drive_force;              // '<Root>/max_drive_force'
  real_T rudder;                       // '<Root>/rudder'
  real_T sail;                         // '<Root>/sail'
  real_T ship_speed;                   // '<Root>/ship_speed'
  real_T speed_angle_d;                // '<Root>/speed_angle_d'
  real_T wind_angle_ground;            // '<Root>/wind_angle_ground'
  real_T wind_speed;                   // '<Root>/wind_speed'
  real_T leg;                          // '<Root>/leg'
  real_T obstacle_judge;               // '<Root>/obstacle_judge'
} ExtY_collision_avoidance_T;

typedef struct {
  std::vector<double> data;             //In message "obs_msg.msg", float64[] is a data structure of vector<double>
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

typedef struct{
  double x;
  double y;
} Obstacle_pose_T;

class scanningModelClass {
  // public data and function members
 public:

  // External inputs
  ExtU_collision_avoidance_T collision_avoidance_U;

  // External outputs
  ExtY_collision_avoidance_T collision_avoidance_Y;

  Obstacle_information_T obstacle_information;

  Static_wind_information_T static_wind_information;

  Target_information_T target_information;

  Obstacle_output_T output;

  Obstacle_pose_T obstacle_pose;

  // Constructor
  scanningModelClass(){};

  // Destructor
  ~scanningModelClass(){};

  //initialization
  void initialize();

  void avoidance_algo();

  /*
  //get rudder angle and sail angle
  double Get_Rudder();
  double Get_Sail();
  int Get_PCCtrl();
  //calculate rudder angle with PID
  void set_Pid();
  void AP_Calc();
  */

 private:

  // private data and function members


};



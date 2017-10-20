//
// File: path_following.h
//
// Code generated for Simulink model 'path_following'.
//
// Model version                  : 1.315
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Fri Oct 20 13:07:25 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_path_following_h_
#define RTW_HEADER_path_following_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef path_following_COMMON_INCLUDES_
# define path_following_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // path_following_COMMON_INCLUDES_

#include "path_following_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  real_T real_wind_history[6000];      // '<Root>/MATLAB Function7'
  real_T price_fixSail[1140];
  real_T Dis[1000];
  real_T ship_speed_history[800];      // '<Root>/MATLAB Function8'
  real_T price[760];
} B_path_following_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay15_DSTATE[300];      // '<Root>/Unit Delay15'
  real_T UnitDelay14_DSTATE[800];      // '<Root>/Unit Delay14'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
  real_T UnitDelay3_DSTATE;            // '<Root>/Unit Delay3'
  real_T UnitDelay4_DSTATE;            // '<Root>/Unit Delay4'
  real_T UnitDelay1_DSTATE;            // '<Root>/Unit Delay1'
  real_T UnitDelay13_DSTATE[6000];     // '<Root>/Unit Delay13'
  real_T UnitDelay8_DSTATE;            // '<Root>/Unit Delay8'
  real_T UnitDelay9_DSTATE;            // '<Root>/Unit Delay9'
  real_T UnitDelay16_DSTATE;           // '<Root>/Unit Delay16'
  real_T UnitDelay10_DSTATE;           // '<Root>/Unit Delay10'
  real_T UnitDelay2_DSTATE;            // '<Root>/Unit Delay2'
  real_T UnitDelay7_DSTATE;            // '<Root>/Unit Delay7'
  real_T UnitDelay6_DSTATE;            // '<Root>/Unit Delay6'
} DW_path_following_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T North;                        // '<Root>/North'
  real_T East;                         // '<Root>/East'
  real_T Yaw;                          // '<Root>/Yaw'
  real_T Roll;                         // '<Root>/Roll'
  real_T Yaw_rate;                     // '<Root>/Yaw_rate'
  real_T Roll_rate;                    // '<Root>/Roll_rate'
  real_T Airmar_wind_angle;            // '<Root>/Airmar_wind_angle'
  real_T Airmar_wind_speed;            // '<Root>/Airmar_wind_speed'
  real_T path[200];                    // '<Root>/path'
  real_T path_num;                     // '<Root>/path_num'
} ExtU_path_following_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T speed_angle;                  // '<Root>/speed_angle'
  real_T los_heading;                  // '<Root>/los_heading'
  real_T sail_ground_d;                // '<Root>/sail_ground_d'
  real_T drive_force;                  // '<Root>/drive_force'
  real_T wind_speed;                   // '<Root>/wind_speed'
  real_T speed_angle_d;                // '<Root>/speed_angle_d'
  real_T HorizontalSpeed;              // '<Root>/Horizontal Speed'
  real_T wind_angle_mean;              // '<Root>/wind_angle_mean'
  real_T rudder;                       // '<Root>/rudder'
  real_T sail;                         // '<Root>/sail'
} ExtY_path_following_T;

// Parameters (auto storage)
struct P_path_following_T_ {
  real_T Kd;                           // Variable: Kd
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Ki;                           // Variable: Ki
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Kp;                           // Variable: Kp
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T R;                            // Variable: R
                                       //  Referenced by: '<Root>/MATLAB Function3'

  real_T jibing_time;                  // Variable: jibing_time
                                       //  Referenced by: '<Root>/MATLAB Function4'

  real_T leg;                          // Variable: leg
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T max_loose_time;               // Variable: max_loose_time
                                       //  Referenced by: '<Root>/MATLAB Function6'

  real_T max_roll_allowed;             // Variable: max_roll_allowed
                                       //  Referenced by: '<Root>/MATLAB Function6'

  real_T pos_history_len;              // Variable: pos_history_len
                                       //  Referenced by: '<Root>/MATLAB Function8'

  real_T run_period;                   // Variable: run_period
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T ship_speed_history_len;       // Variable: ship_speed_history_len
                                       //  Referenced by: '<Root>/MATLAB Function8'

  real_T tacking_discount_decrease_windspeed;// Variable: tacking_discount_decrease_windspeed
                                             //  Referenced by: '<Root>/MATLAB Function1'

  real_T tacking_force_discount;       // Variable: tacking_force_discount
                                       //  Referenced by: '<Root>/MATLAB Function1'

  real_T tacking_time;                 // Variable: tacking_time
                                       //  Referenced by: '<Root>/MATLAB Function4'

  real_T upwind_R_expand_ratio;        // Variable: upwind_R_expand_ratio
                                       //  Referenced by: '<Root>/MATLAB Function3'

  real_T wind_mean_time;               // Variable: wind_mean_time
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T Airmar_X_Value;               // Expression: -0.61
                                       //  Referenced by: '<Root>/Airmar_X'

  real_T Airmar_Z_Value;               // Expression: 0.8
                                       //  Referenced by: '<Root>/Airmar_Z'

  real_T UnitDelay15_InitialCondition[300];// Expression: zeros(100,3)
                                           //  Referenced by: '<Root>/Unit Delay15'

  real_T UnitDelay14_InitialCondition[800];// Expression: zeros(400,2)
                                           //  Referenced by: '<Root>/Unit Delay14'

  real_T UnitDelay5_InitialCondition;  // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay5'

  real_T UnitDelay3_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay3'

  real_T UnitDelay4_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay4'

  real_T UnitDelay1_InitialCondition;  // Expression: 0.1
                                       //  Referenced by: '<Root>/Unit Delay1'

  real_T UnitDelay13_InitialCondition[6000];// Expression: zeros(2000,3)
                                            //  Referenced by: '<Root>/Unit Delay13'

  real_T UnitDelay8_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay8'

  real_T UnitDelay9_InitialCondition;  // Expression: pi/2
                                       //  Referenced by: '<Root>/Unit Delay9'

  real_T UnitDelay16_InitialCondition; // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay16'

  real_T UnitDelay10_InitialCondition; // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay10'

  real_T UnitDelay2_InitialCondition;  // Expression: -1
                                       //  Referenced by: '<Root>/Unit Delay2'

  real_T UnitDelay7_InitialCondition;  // Expression: pi/2
                                       //  Referenced by: '<Root>/Unit Delay7'

  real_T UnitDelay6_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay6'

};

// Real-time Model Data Structure
struct tag_RTM_path_following_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model path_following
class scanningModelClass {
  // public data and function members
 public:
  // Tunable parameters
  P_path_following_T path_following_P;

  // Block signals
  B_path_following_T path_following_B;

  // Block states
  DW_path_following_T path_following_DW;

  // External inputs
  ExtU_path_following_T path_following_U;

  // External outputs
  ExtY_path_following_T path_following_Y;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  scanningModelClass();

  // Destructor
  ~scanningModelClass();

  // Real-Time Model get method
  RT_MODEL_path_following_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_path_following_T path_following_M;

  // private member function(s) for subsystem '<Root>'
  real_T path_following_GetAngle(real_T x, real_T y);
  real_T path_following_AngleDiff_l(real_T angle1, real_T angle2);
  real_T path_following_AngleDiff(real_T angle1, real_T angle2);
  real_T path_following_AngleDiff_g(real_T angle1, real_T angle2);
  real_T path_following_SailDeadZone(real_T sail_check, const real_T
    dead_sail_data[], const int32_T dead_sail_sizes[2]);
  real_T path_following_ppval(const real_T pp_breaks[59], const real_T pp_coefs
    [232], real_T x);
  real_T path_following_interp1(real_T varargin_3);
  real_T path_following_interp1_l(real_T varargin_3);
  real_T path_following_interp1_ls(real_T varargin_3);
  real_T path_following_interiorSlope(real_T d1, real_T d2, real_T w1, real_T w2);
  real_T path_following_interp1_lsv(const real_T varargin_2[5], real_T
    varargin_3);
  void path_follow_getSailForce_ground(real_T WindAngle_ground, real_T
    SailAngle_ground, real_T WindSpeed, real_T *SailForce, real_T
    *SailForceAngle_ground, real_T *Attack_angle);
  real_T path_following_HeadingDeadZone(real_T heading_check, real_T
    WindAngle_ground, real_T SailAngle_ground, real_T tacking, real_T
    heading_d_last);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'path_following'
//  '<S1>'   : 'path_following/MATLAB Function1'
//  '<S2>'   : 'path_following/MATLAB Function2'
//  '<S3>'   : 'path_following/MATLAB Function3'
//  '<S4>'   : 'path_following/MATLAB Function4'
//  '<S5>'   : 'path_following/MATLAB Function6'
//  '<S6>'   : 'path_following/MATLAB Function7'
//  '<S7>'   : 'path_following/MATLAB Function8'
//  '<S8>'   : 'path_following/MATLAB Function9'

#endif                                 // RTW_HEADER_path_following_h_

//
// File trailer for generated code.
//
// [EOF]
//

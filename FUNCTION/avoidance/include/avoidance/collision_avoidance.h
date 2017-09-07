//
// File: collision_avoidance.h
//
// Code generated for Simulink model 'collision_avoidance'.
//
// Model version                  : 1.295
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Wed Sep 06 22:21:04 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_collision_avoidance_h_
#define RTW_HEADER_collision_avoidance_h_
#include <cmath>
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef collision_avoidance_COMMON_INCLUDES_
# define collision_avoidance_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // collision_avoidance_COMMON_INCLUDES_

#include "collision_avoidance_types.h"
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
  real_T real_wind_history[6000];      // '<Root>/MATLAB Function6'
  real_T price_fixSail[1140];
  real_T Dis[1000];
  real_T ship_speed_history[800];      // '<Root>/MATLAB Function7'
  real_T price[760];
} B_collision_avoidance_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay17_DSTATE;           // '<Root>/Unit Delay17'
  real_T UnitDelay18_DSTATE[500];      // '<Root>/Unit Delay18'
  real_T UnitDelay20_DSTATE;           // '<Root>/Unit Delay20'
  real_T UnitDelay12_DSTATE[2];        // '<Root>/Unit Delay12'
  real_T UnitDelay16_DSTATE;           // '<Root>/Unit Delay16'
  real_T UnitDelay4_DSTATE;            // '<Root>/Unit Delay4'
  real_T UnitDelay1_DSTATE;            // '<Root>/Unit Delay1'
  real_T UnitDelay13_DSTATE[6000];     // '<Root>/Unit Delay13'
  real_T UnitDelay15_DSTATE[300];      // '<Root>/Unit Delay15'
  real_T UnitDelay14_DSTATE[800];      // '<Root>/Unit Delay14'
  real_T UnitDelay7_DSTATE;            // '<Root>/Unit Delay7'
  real_T UnitDelay8_DSTATE;            // '<Root>/Unit Delay8'
  real_T UnitDelay9_DSTATE;            // '<Root>/Unit Delay9'
  real_T UnitDelay10_DSTATE;           // '<Root>/Unit Delay10'
  real_T UnitDelay11_DSTATE;           // '<Root>/Unit Delay11'
  real_T UnitDelay6_DSTATE;            // '<Root>/Unit Delay6'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
} DW_collision_avoidance_T;

// External inputs (root inport signals with auto storage)
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

// External outputs (root outports fed by signals with auto storage)
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

// Parameters (auto storage)
struct P_collision_avoidance_T_ {
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

  real_T judge_step;                   // Variable: judge_step
                                       //  Referenced by: '<Root>/MATLAB Function10'

  real_T max_loose_time;               // Variable: max_loose_time
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T max_roll_allowed;             // Variable: max_roll_allowed
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T obstacle_proba_threshold;     // Variable: obstacle_proba_threshold
                                       //  Referenced by: '<Root>/MATLAB Function10'

  real_T points[8];                    // Variable: points
                                       //  Referenced by: '<Root>/MATLAB Function'

  real_T pos_history_len;              // Variable: pos_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T run_period;                   // Variable: run_period
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T ship_speed_history_len;       // Variable: ship_speed_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T start_counting;               // Variable: start_counting
                                       //  Referenced by: '<Root>/MATLAB Function'

  real_T tacking_discount_decrease_windspeed;// Variable: tacking_discount_decrease_windspeed
                                             //  Referenced by: '<Root>/MATLAB Function1'

  real_T tacking_force_discount;       // Variable: tacking_force_discount
                                       //  Referenced by: '<Root>/MATLAB Function1'

  real_T tacking_speed;                // Variable: tacking_speed
                                       //  Referenced by: '<Root>/MATLAB Function4'

  real_T tacking_time;                 // Variable: tacking_time
                                       //  Referenced by: '<Root>/MATLAB Function4'

  real_T upwind_R_expand_ratio;        // Variable: upwind_R_expand_ratio
                                       //  Referenced by: '<Root>/MATLAB Function3'

  real_T wind_mean_time;               // Variable: wind_mean_time
                                       //  Referenced by: '<Root>/MATLAB Function6'

  real_T wind_side;                    // Variable: wind_side
                                       //  Referenced by: '<Root>/MATLAB Function'

  real_T Airmar_X_Value;               // Expression: -0.61
                                       //  Referenced by: '<Root>/Airmar_X'

  real_T Airmar_Z_Value;               // Expression: 0.8
                                       //  Referenced by: '<Root>/Airmar_Z'

  real_T UnitDelay17_InitialCondition; // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay17'

  real_T UnitDelay18_InitialCondition[500];// Expression: zeros([500,1])
                                           //  Referenced by: '<Root>/Unit Delay18'

  real_T UnitDelay20_InitialCondition; // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay20'

  real_T UnitDelay12_InitialCondition[2];// Expression: [0,0.5]
                                         //  Referenced by: '<Root>/Unit Delay12'

  real_T UnitDelay16_InitialCondition; // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay16'

  real_T UnitDelay19_InitialCondition; // Expression: 0.5
                                       //  Referenced by: '<Root>/Unit Delay19'

  real_T UnitDelay4_InitialCondition;  // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay4'

  real_T UnitDelay2_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay2'

  real_T UnitDelay3_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay3'

  real_T UnitDelay1_InitialCondition;  // Expression: 0.1
                                       //  Referenced by: '<Root>/Unit Delay1'

  real_T UnitDelay13_InitialCondition[6000];// Expression: zeros(2000,3)
                                            //  Referenced by: '<Root>/Unit Delay13'

  real_T UnitDelay15_InitialCondition[300];// Expression: zeros(100,3)
                                           //  Referenced by: '<Root>/Unit Delay15'

  real_T UnitDelay14_InitialCondition[800];// Expression: zeros(400,2)
                                           //  Referenced by: '<Root>/Unit Delay14'

  real_T UnitDelay7_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay7'

  real_T UnitDelay8_InitialCondition;  // Expression: pi/2
                                       //  Referenced by: '<Root>/Unit Delay8'

  real_T UnitDelay9_InitialCondition;  // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay9'

  real_T UnitDelay10_InitialCondition; // Expression: 1
                                       //  Referenced by: '<Root>/Unit Delay10'

  real_T UnitDelay11_InitialCondition; // Expression: -1
                                       //  Referenced by: '<Root>/Unit Delay11'

  real_T UnitDelay6_InitialCondition;  // Expression: pi/2
                                       //  Referenced by: '<Root>/Unit Delay6'

  real_T UnitDelay5_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay5'

};

// Real-time Model Data Structure
struct tag_RTM_collision_avoidance_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model collision_avoidance
class scanningModelClass {
  // public data and function members
 public:
  // Tunable parameters
  P_collision_avoidance_T collision_avoidance_P;

  // Block signals
  B_collision_avoidance_T collision_avoidance_B;

  // Block states
  DW_collision_avoidance_T collision_avoidance_DW;

  // External inputs
  ExtU_collision_avoidance_T collision_avoidance_U;

  // External outputs
  ExtY_collision_avoidance_T collision_avoidance_Y;

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
  RT_MODEL_collision_avoidance_T * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_collision_avoidance_T collision_avoidance_M;

  // private member function(s) for subsystem '<Root>'
  real_T collision_avoidance_GetAngle(real_T x, real_T y);
  real_T collision_avoidance_AngleDiff(real_T angle1, real_T angle2);
  real_T collision_avoidance_GetAngle_n(real_T x, real_T y);
  real_T collision_avoidance_AngleDiff_b(real_T angle1, real_T angle2);
  real_T collision_avoidance_AngleDiff_f(real_T angle1, real_T angle2);
  real_T collision_avoidance_AngleDiff_p(real_T angle1, real_T angle2);
  real_T collision_avoidanc_SailDeadZone(real_T sail_check, const real_T
    dead_sail_data[], const int32_T dead_sail_sizes[2]);
  real_T collision_avoidance_ppval(const real_T pp_breaks[59], const real_T
    pp_coefs[232], real_T x);
  real_T collision_avoidance_interp1(real_T varargin_3);
  real_T collision_avoidance_interp1_n(real_T varargin_3);
  real_T collision_avoidance_interp1_nz(real_T varargin_3);
  real_T collision_avoidan_interiorSlope(real_T d1, real_T d2, real_T w1, real_T
    w2);
  real_T collision_avoidance_interp1_nz3(const real_T varargin_2[5], real_T
    varargin_3);
  void collision_a_getSailForce_ground(real_T WindAngle_ground, real_T
    SailAngle_ground, real_T WindSpeed, real_T *SailForce, real_T
    *SailForceAngle_ground, real_T *Attack_angle);
  real_T collision_avoid_HeadingDeadZone(real_T heading_check, real_T
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
//  '<Root>' : 'collision_avoidance'
//  '<S1>'   : 'collision_avoidance/MATLAB Function'
//  '<S2>'   : 'collision_avoidance/MATLAB Function1'
//  '<S3>'   : 'collision_avoidance/MATLAB Function10'
//  '<S4>'   : 'collision_avoidance/MATLAB Function2'
//  '<S5>'   : 'collision_avoidance/MATLAB Function3'
//  '<S6>'   : 'collision_avoidance/MATLAB Function4'
//  '<S7>'   : 'collision_avoidance/MATLAB Function5'
//  '<S8>'   : 'collision_avoidance/MATLAB Function6'
//  '<S9>'   : 'collision_avoidance/MATLAB Function7'
//  '<S10>'  : 'collision_avoidance/MATLAB Function8'

#endif                                 // RTW_HEADER_collision_avoidance_h_

//
// File trailer for generated code.
//
// [EOF]
//

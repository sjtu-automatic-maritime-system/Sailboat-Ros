//
// File: race_course.h
//
// Code generated for Simulink model 'race_course'.
//
// Model version                  : 1.311
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Thu Jul 06 13:08:27 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_race_course_h_
#define RTW_HEADER_race_course_h_
#include <cmath>
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef race_course_COMMON_INCLUDES_
# define race_course_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // race_course_COMMON_INCLUDES_

#include "race_course_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rtGetInf.h"

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
  real_T ship_speed_history[800];      // '<Root>/MATLAB Function7'
  real_T price[760];
} B_race_course_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay4_DSTATE;            // '<Root>/Unit Delay4'
  real_T UnitDelay13_DSTATE[6000];     // '<Root>/Unit Delay13'
  real_T UnitDelay15_DSTATE[300];      // '<Root>/Unit Delay15'
  real_T UnitDelay14_DSTATE[800];      // '<Root>/Unit Delay14'
  real_T UnitDelay7_DSTATE;            // '<Root>/Unit Delay7'
  real_T UnitDelay8_DSTATE;            // '<Root>/Unit Delay8'
  real_T UnitDelay9_DSTATE;            // '<Root>/Unit Delay9'
  real_T UnitDelay10_DSTATE;           // '<Root>/Unit Delay10'
  real_T UnitDelay1_DSTATE;            // '<Root>/Unit Delay1'
  real_T UnitDelay6_DSTATE;            // '<Root>/Unit Delay6'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
} DW_race_course_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T North;                        // '<Root>/North'
  real_T East;                         // '<Root>/East'
  real_T Yaw;                          // '<Root>/Yaw'
  real_T Roll;                         // '<Root>/Roll'
  real_T Roll_rate;                    // '<Root>/Roll_rate'
  real_T Yaw_rate;                     // '<Root>/Yaw_rate'
  real_T Airmar_wind_angle;            // '<Root>/Airmar_wind_angle'
  real_T Airmar_wind_speed;            // '<Root>/Airmar_wind_speed'
} ExtU_race_course_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T Sail;                         // '<Root>/Sail'
  real_T Rudder;                       // '<Root>/Rudder'
  real_T speed_angle_d;                // '<Root>/speed_angle_d'
  real_T speed_angle;                  // '<Root>/speed_angle'
  real_T wind_speed;                   // '<Root>/wind_speed'
  real_T wind_angle_ground;            // '<Root>/wind_angle_ground'
  real_T leg;                          // '<Root>/leg'
  real_T los_heading;                  // '<Root>/los_heading'
} ExtY_race_course_T;

// Parameters (auto storage)
struct P_race_course_T_ {
  real_T Kd;                           // Variable: Kd
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Ki;                           // Variable: Ki
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Kp;                           // Variable: Kp
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T R_reach;                      // Variable: R_reach
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T R_reach_big;                  // Variable: R_reach_big
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T extra_leg_len;                // Variable: extra_leg_len
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T max_loose_time;               // Variable: max_loose_time
                                       //  Referenced by: '<Root>/MATLAB Function9'

  real_T max_roll_allowed;             // Variable: max_roll_allowed
                                       //  Referenced by: '<Root>/MATLAB Function9'

  real_T pos_history_len;              // Variable: pos_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T race_points[8];               // Variable: race_points
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T run_period;                   // Variable: run_period
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T ship_speed_history_len;       // Variable: ship_speed_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T upwind_leg;                   // Variable: upwind_leg
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T wind_mean_time;               // Variable: wind_mean_time
                                       //  Referenced by: '<Root>/MATLAB Function6'

};

// Real-time Model Data Structure
struct tag_RTM_race_course_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model race_course
class race_courseModelClass {
  // public data and function members
 public:
  // Tunable parameters
  P_race_course_T race_course_P;

  // External inputs
  ExtU_race_course_T race_course_U;

  // External outputs
  ExtY_race_course_T race_course_Y;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  race_courseModelClass();

  // Destructor
  ~race_courseModelClass();

  // Real-Time Model get method
  RT_MODEL_race_course_T * getRTM();

  // private data and function members
 private:
  // Block signals
  B_race_course_T race_course_B;

  // Block states
  DW_race_course_T race_course_DW;

  // Real-Time Model
  RT_MODEL_race_course_T race_course_M;

  // private member function(s) for subsystem '<Root>'
  real_T race_course_AngleDiff(real_T angle1, real_T angle2);
  real_T race_course_AngleDiff_p(real_T angle1, real_T angle2);
  real_T race_course_SailDeadZone(real_T sail_check, const real_T
    dead_sail_data[], const int32_T dead_sail_sizes[2]);
  real_T race_course_ppval(const real_T pp_breaks[59], const real_T pp_coefs[232],
    real_T x);
  real_T race_course_interp1(real_T varargin_3);
  real_T race_course_interp1_n(real_T varargin_3);
  real_T race_course_interp1_nz(real_T varargin_3);
  real_T race_course_interiorSlope(real_T d1, real_T d2, real_T w1, real_T w2);
  real_T race_course_interp1_nz3(const real_T varargin_2[5], real_T varargin_3);
  void race_course_getSailForce_ground(real_T WindAngle_ground, real_T
    SailAngle_ground, real_T WindSpeed, real_T *SailForce, real_T
    *SailForceAngle_ground, real_T *Attack_angle);
  real_T race_course_HeadingDeadZone(real_T heading_check, real_T
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
//  '<Root>' : 'race_course'
//  '<S1>'   : 'race_course/MATLAB Function1'
//  '<S2>'   : 'race_course/MATLAB Function2'
//  '<S3>'   : 'race_course/MATLAB Function3'
//  '<S4>'   : 'race_course/MATLAB Function4'
//  '<S5>'   : 'race_course/MATLAB Function5'
//  '<S6>'   : 'race_course/MATLAB Function6'
//  '<S7>'   : 'race_course/MATLAB Function7'
//  '<S8>'   : 'race_course/MATLAB Function8'
//  '<S9>'   : 'race_course/MATLAB Function9'

#endif                                 // RTW_HEADER_race_course_h_

//
// File trailer for generated code.
//
// [EOF]
//

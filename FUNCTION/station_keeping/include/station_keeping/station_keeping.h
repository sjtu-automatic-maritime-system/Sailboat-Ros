//
// File: station_keeping.h
//
// Code generated for Simulink model 'station_keeping'.
//
// Model version                  : 1.261
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Thu Jul 06 15:10:39 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_station_keeping_h_
#define RTW_HEADER_station_keeping_h_
#include <cmath>
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef station_keeping_COMMON_INCLUDES_
# define station_keeping_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // station_keeping_COMMON_INCLUDES_

#include "station_keeping_types.h"
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
} B_station_keeping_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay15_DSTATE[300];      // '<Root>/Unit Delay15'
  real_T UnitDelay14_DSTATE[800];      // '<Root>/Unit Delay14'
  real_T UnitDelay13_DSTATE[6000];     // '<Root>/Unit Delay13'
  real_T UnitDelay7_DSTATE;            // '<Root>/Unit Delay7'
  real_T UnitDelay8_DSTATE;            // '<Root>/Unit Delay8'
  real_T UnitDelay9_DSTATE;            // '<Root>/Unit Delay9'
  real_T UnitDelay11_DSTATE;           // '<Root>/Unit Delay11'
  real_T UnitDelay6_DSTATE;            // '<Root>/Unit Delay6'
  real_T UnitDelay5_DSTATE;            // '<Root>/Unit Delay5'
} DW_station_keeping_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T North;                        // '<Root>/North'
  real_T East;                         // '<Root>/East'
  real_T ahrs_Yaw;                     // '<Root>/ahrs_Yaw'
  real_T ahrs_Roll;                    // '<Root>/ahrs_Roll'
  real_T ahrs_Roll_rate;               // '<Root>/ahrs_Roll_rate'
  real_T ahrs_Yaw_rate;                // '<Root>/ahrs_Yaw_rate'
  real_T Airmar_wind_angle;            // '<Root>/Airmar_wind_angle'
  real_T Airmar_wind_speed;            // '<Root>/Airmar_wind_speed'
} ExtU_station_keeping_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T rudder;                       // '<Root>/rudder'
  real_T sail;                         // '<Root>/sail'
  real_T wind_speed;                   // '<Root>/wind_speed'
  real_T wind_angle_ground;            // '<Root>/wind_angle_ground'
  real_T speed_angle;                  // '<Root>/speed_angle'
  real_T speed_angle_d;                // '<Root>/speed_angle_d'
  real_T los_heading;                  // '<Root>/los_heading'
} ExtY_station_keeping_T;

// Parameters (auto storage)
struct P_station_keeping_T_ {
  real_T Kd;                           // Variable: Kd
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Ki;                           // Variable: Ki
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T Kp;                           // Variable: Kp
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T max_loose_time;               // Variable: max_loose_time
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T max_roll_allowed;             // Variable: max_roll_allowed
                                       //  Referenced by: '<Root>/MATLAB Function5'

  real_T point_keeping[2];             // Variable: point_keeping
                                       //  Referenced by: '<Root>/MATLAB Function'

  real_T pos_history_len;              // Variable: pos_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T run_period;                   // Variable: run_period
                                       //  Referenced by: '<Root>/MATLAB Function2'

  real_T ship_speed_history_len;       // Variable: ship_speed_history_len
                                       //  Referenced by: '<Root>/MATLAB Function7'

  real_T tacking_force_discount;       // Variable: tacking_force_discount
                                       //  Referenced by: '<Root>/MATLAB Function1'

  real_T wind_mean_time;               // Variable: wind_mean_time
                                       //  Referenced by: '<Root>/MATLAB Function6'

};

// Real-time Model Data Structure
struct tag_RTM_station_keeping_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model station_keeping
class station_keepingModelClass {
  // public data and function members
 public:
  // Tunable parameters
  P_station_keeping_T station_keeping_P;

  // External inputs
  ExtU_station_keeping_T station_keeping_U;

  // External outputs
  ExtY_station_keeping_T station_keeping_Y;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  station_keepingModelClass();

  // Destructor
  ~station_keepingModelClass();

  // Real-Time Model get method
  RT_MODEL_station_keeping_T * getRTM();

  // private data and function members
 private:
  // Block signals
  B_station_keeping_T station_keeping_B;

  // Block states
  DW_station_keeping_T station_keeping_DW;

  // Real-Time Model
  RT_MODEL_station_keeping_T station_keeping_M;

  // private member function(s) for subsystem '<Root>'
  real_T station_keeping_AngleDiff(real_T angle1, real_T angle2);
  real_T station_keeping_AngleDiff_p(real_T angle1, real_T angle2);
  real_T station_keeping_SailDeadZone(real_T sail_check, const real_T
    dead_sail_data[], const int32_T dead_sail_sizes[2]);
  real_T station_keeping_ppval(const real_T pp_breaks[59], const real_T
    pp_coefs[232], real_T x);
  real_T station_keeping_interp1(real_T varargin_3);
  real_T station_keeping_interp1_n(real_T varargin_3);
  real_T station_keeping_interp1_nz(real_T varargin_3);
  real_T station_keeping_interiorSlope(real_T d1, real_T d2, real_T w1, real_T
    w2);
  real_T station_keeping_interp1_nz3(const real_T varargin_2[5], real_T
    varargin_3);
  void station_kee_getSailForce_ground(real_T WindAngle_ground, real_T
    SailAngle_ground, real_T WindSpeed, real_T *SailForce, real_T
    *SailForceAngle_ground, real_T *Attack_angle);
  real_T station_keeping_HeadingDeadZone(real_T heading_check, real_T
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
//  '<Root>' : 'station_keeping'
//  '<S1>'   : 'station_keeping/MATLAB Function'
//  '<S2>'   : 'station_keeping/MATLAB Function1'
//  '<S3>'   : 'station_keeping/MATLAB Function2'
//  '<S4>'   : 'station_keeping/MATLAB Function4'
//  '<S5>'   : 'station_keeping/MATLAB Function5'
//  '<S6>'   : 'station_keeping/MATLAB Function6'
//  '<S7>'   : 'station_keeping/MATLAB Function7'
//  '<S8>'   : 'station_keeping/MATLAB Function8'

#endif                                 // RTW_HEADER_station_keeping_h_

//
// File trailer for generated code.
//
// [EOF]
//

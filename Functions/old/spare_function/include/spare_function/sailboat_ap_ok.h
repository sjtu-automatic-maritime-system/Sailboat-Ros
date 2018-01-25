//
// File: sailboat_ap_ok.h
//
// Code generated for Simulink model 'sailboat_ap_ok'.
//
// Model version                  : 1.226
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Sun Oct 08 15:19:58 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_sailboat_ap_ok_h_
#define RTW_HEADER_sailboat_ap_ok_h_
#include "rtwtypes.h"
#include <cmath>
#include <stddef.h>
#include <string.h>
#ifndef sailboat_ap_ok_COMMON_INCLUDES_
# define sailboat_ap_ok_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // sailboat_ap_ok_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals for system '<Root>/inf to pi'
typedef struct {
  real_T a;                            // '<Root>/inf to pi'
} B_inftopi;

// Block signals (auto storage)
typedef struct {
  B_inftopi sf_inftopi1;               // '<Root>/inf to pi1'
  B_inftopi sf_inftopi;                // '<Root>/inf to pi'
} B;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;// '<S5>/Discrete-Time Integrator'
  real_T UD_DSTATE;                    // '<S7>/UD'
  real_T DiscreteTimeIntegrator_DSTATE_j;// '<S1>/Discrete-Time Integrator'
  real_T UD_DSTATE_o;                  // '<S6>/UD'
  real_T Memory1_PreviousInput;        // '<S5>/Memory1'
  real_T Memory_PreviousInput;         // '<S1>/Memory'
} DW;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T roll;                         // '<Root>/roll'
  real_T yaw;                          // '<Root>/yaw'
  real_T relative_wind;                // '<Root>/relative_wind'
} ExtU;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T Rudder;                       // '<Root>/Rudder'
  real_T Sail;                         // '<Root>/Sail'
} ExtY;

// Parameters (auto storage)
struct P_ {
  real_T oyaw;                         // Variable: oyaw
                                       //  Referenced by: '<Root>/Oyaw'

  real_T rudderD;                      // Variable: rudderD
                                       //  Referenced by: '<S5>/rudderD'

  real_T rudderI;                      // Variable: rudderI
                                       //  Referenced by: '<S5>/rudderI'

  real_T rudderP;                      // Variable: rudderP
                                       //  Referenced by: '<S5>/rudderP'

  real_T sailD;                        // Variable: sailD
                                       //  Referenced by: '<S1>/sailD'

  real_T sailI;                        // Variable: sailI
                                       //  Referenced by: '<S1>/sailI'

  real_T sailP;                        // Variable: sailP
                                       //  Referenced by: '<S1>/sailP'

  real_T DiscreteDerivative_ICPrevScaled;// Mask Parameter: DiscreteDerivative_ICPrevScaled
                                         //  Referenced by: '<S6>/UD'

  real_T DiscreteDerivative_ICPrevScal_g;// Mask Parameter: DiscreteDerivative_ICPrevScal_g
                                         //  Referenced by: '<S7>/UD'

  real_T Constant3_Value;              // Expression: 1
                                       //  Referenced by: '<S1>/Constant3'

  real_T Constant2_Value;              // Expression: 1.2
                                       //  Referenced by: '<S1>/Constant2'

  real_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                        //  Referenced by: '<S1>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S6>/TSamp'

  real_T Memory_X0;                    // Expression: 0
                                       //  Referenced by: '<S1>/Memory'

  real_T Switch1_Threshold;            // Expression: 0.25
                                       //  Referenced by: '<S1>/Switch1'

  real_T Saturation1_UpperSat;         // Expression: 0.20
                                       //  Referenced by: '<S1>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: -0.20
                                       //  Referenced by: '<S1>/Saturation1'

  real_T Saturation3_UpperSat;         // Expression: 1.39
                                       //  Referenced by: '<S1>/Saturation3'

  real_T Saturation3_LowerSat;         // Expression: -1.39
                                       //  Referenced by: '<S1>/Saturation3'

  real_T Memory1_X0;                   // Expression: 0
                                       //  Referenced by: '<S5>/Memory1'

  real_T DiscreteTimeIntegrator_gainva_h;// Computed Parameter: DiscreteTimeIntegrator_gainva_h
                                         //  Referenced by: '<S5>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC_e;  // Expression: 0
                                       //  Referenced by: '<S5>/Discrete-Time Integrator'

  real_T TSamp_WtEt_i;                 // Computed Parameter: TSamp_WtEt_i
                                       //  Referenced by: '<S7>/TSamp'

  real_T Saturation_UpperSat;          // Expression: 0.175
                                       //  Referenced by: '<S5>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -0.175
                                       //  Referenced by: '<S5>/Saturation'

  real_T Saturation2_UpperSat;         // Expression: 0.68
                                       //  Referenced by: '<S5>/Saturation2'

  real_T Saturation2_LowerSat;         // Expression: -0.68
                                       //  Referenced by: '<S5>/Saturation2'

  real_T Saturation4_UpperSat;         // Expression: 0.68
                                       //  Referenced by: '<Root>/Saturation4'

  real_T Saturation4_LowerSat;         // Expression: -0.68
                                       //  Referenced by: '<Root>/Saturation4'

};

// Parameters (auto storage)
typedef struct P_ P;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model sailboat_ap_ok
class sailboat_ap_okModelClass {
  // public data and function members
 public:
  // Tunable parameters
  P rtP;

  // Block signals
  B rtB;

  // Block states
  DW rtDW;

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  sailboat_ap_okModelClass();

  // Destructor
  ~sailboat_ap_okModelClass();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S6>/Data Type Duplicate' : Unused code path elimination
//  Block '<S1>/Display7' : Unused code path elimination
//  Block '<S1>/Display8' : Unused code path elimination
//  Block '<S1>/Scope' : Unused code path elimination
//  Block '<S1>/Scope1' : Unused code path elimination
//  Block '<Root>/Display6' : Unused code path elimination
//  Block '<S7>/Data Type Duplicate' : Unused code path elimination
//  Block '<S5>/Display10' : Unused code path elimination
//  Block '<S5>/Display11' : Unused code path elimination
//  Block '<S5>/Display4' : Unused code path elimination


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
//  '<Root>' : 'sailboat_ap_ok'
//  '<S1>'   : 'sailboat_ap_ok/ Slove Mdeg'
//  '<S2>'   : 'sailboat_ap_ok/Oyaw'
//  '<S3>'   : 'sailboat_ap_ok/inf to pi'
//  '<S4>'   : 'sailboat_ap_ok/inf to pi1'
//  '<S5>'   : 'sailboat_ap_ok/slove Rdeg'
//  '<S6>'   : 'sailboat_ap_ok/ Slove Mdeg/Discrete Derivative'
//  '<S7>'   : 'sailboat_ap_ok/slove Rdeg/Discrete Derivative'
//  '<S8>'   : 'sailboat_ap_ok/slove Rdeg/MATLAB Function2'

#endif                                 // RTW_HEADER_sailboat_ap_ok_h_

//
// File trailer for generated code.
//
// [EOF]
//

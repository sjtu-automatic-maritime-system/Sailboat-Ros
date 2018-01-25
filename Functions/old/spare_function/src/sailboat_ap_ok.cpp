//
// File: sailboat_ap_ok.cpp
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
#include "sailboat_ap_ok.h"

extern void inftopi(real_T rtu_u, B_inftopi *localB);

//
// Output and update for atomic system:
//    '<Root>/inf to pi'
//    '<Root>/inf to pi1'
//
void inftopi(real_T rtu_u, B_inftopi *localB)
{
  real_T u;
  u = rtu_u;

  // MATLAB Function 'inf to pi': '<S3>:1'
  // #chushihua
  while (std::abs(u) > 3.1415926535897931) {
    // '<S3>:1:3'
    if (u > 3.1415926535897931) {
      // '<S3>:1:4'
      // '<S3>:1:5'
      u -= 6.2831853071795862;
    } else {
      if (u < -3.1415926535897931) {
        // '<S3>:1:6'
        // '<S3>:1:7'
        u += 6.2831853071795862;
      }
    }
  }

  // '<S3>:1:10'
  localB->a = u;
}

// Model step function
void sailboat_ap_okModelClass::step()
{
  real_T rtb_Saturation2;
  real_T rtb_TSamp;
  real_T rtb_Oyaw;

  // MATLAB Function: '<Root>/Oyaw'
  // MATLAB Function 'Oyaw': '<S2>:1'
  // '<S2>:1:2'
  rtb_Oyaw = rtP.oyaw;

  // MATLAB Function: '<Root>/inf to pi1' incorporates:
  //   Inport: '<Root>/yaw'

  inftopi(rtU.yaw, &rtB.sf_inftopi1);

  // Outputs for Atomic SubSystem: '<Root>/slove Rdeg'
  // Sum: '<S5>/Odeg-ship'
  rtb_Oyaw -= rtB.sf_inftopi1.a;

  // MATLAB Function: '<S5>/MATLAB Function2'
  // MATLAB Function 'slove Rdeg/MATLAB Function2': '<S8>:1'
  if (rtb_Oyaw > 3.1415926535897931) {
    // '<S8>:1:3'
    // '<S8>:1:4'
    rtb_Oyaw -= 6.2831853071795862;
  } else if (rtb_Oyaw < -3.1415926535897931) {
    // '<S8>:1:5'
    // '<S8>:1:6'
    rtb_Oyaw += 6.2831853071795862;
  } else {
    // '<S8>:1:8'
  }

  // End of MATLAB Function: '<S5>/MATLAB Function2'

  // SampleTimeMath: '<S7>/TSamp' incorporates:
  //   Gain: '<S5>/rudderD'
  //
  //  About '<S7>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp = rtP.rudderD * rtb_Oyaw * rtP.TSamp_WtEt_i;

  // Sum: '<S5>/Subtract' incorporates:
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   Gain: '<S5>/rudderP'
  //   Memory: '<S5>/Memory1'
  //   Sum: '<S5>/Add1'
  //   Sum: '<S7>/Diff'
  //   UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtb_Saturation2 = rtDW.Memory1_PreviousInput - ((rtP.rudderP * rtb_Oyaw +
    rtDW.DiscreteTimeIntegrator_DSTATE) + (rtb_TSamp - rtDW.UD_DSTATE));

  // Saturate: '<S5>/Saturation'
  if (rtb_Saturation2 > rtP.Saturation_UpperSat) {
    rtb_Saturation2 = rtP.Saturation_UpperSat;
  } else {
    if (rtb_Saturation2 < rtP.Saturation_LowerSat) {
      rtb_Saturation2 = rtP.Saturation_LowerSat;
    }
  }

  // Sum: '<S5>/Add' incorporates:
  //   Memory: '<S5>/Memory1'
  //   Saturate: '<S5>/Saturation'

  rtb_Saturation2 = rtDW.Memory1_PreviousInput - rtb_Saturation2;

  // Saturate: '<S5>/Saturation2'
  if (rtb_Saturation2 > rtP.Saturation2_UpperSat) {
    rtb_Saturation2 = rtP.Saturation2_UpperSat;
  } else {
    if (rtb_Saturation2 < rtP.Saturation2_LowerSat) {
      rtb_Saturation2 = rtP.Saturation2_LowerSat;
    }
  }

  // End of Saturate: '<S5>/Saturation2'

  // Update for Memory: '<S5>/Memory1'
  rtDW.Memory1_PreviousInput = rtb_Saturation2;

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S5>/rudderI'

  rtDW.DiscreteTimeIntegrator_DSTATE += rtP.rudderI * rtb_Oyaw *
    rtP.DiscreteTimeIntegrator_gainva_h;

  // Update for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE = rtb_TSamp;

  // End of Outputs for SubSystem: '<Root>/slove Rdeg'

  // Saturate: '<Root>/Saturation4'
  if (rtb_Saturation2 > rtP.Saturation4_UpperSat) {
    // Outport: '<Root>/Rudder'
    rtY.Rudder = rtP.Saturation4_UpperSat;
  } else if (rtb_Saturation2 < rtP.Saturation4_LowerSat) {
    // Outport: '<Root>/Rudder'
    rtY.Rudder = rtP.Saturation4_LowerSat;
  } else {
    // Outport: '<Root>/Rudder'
    rtY.Rudder = rtb_Saturation2;
  }

  // End of Saturate: '<Root>/Saturation4'

  // MATLAB Function: '<Root>/inf to pi' incorporates:
  //   Inport: '<Root>/relative_wind'

  inftopi(rtU.relative_wind, &rtB.sf_inftopi);

  // Outputs for Atomic SubSystem: '<Root>/ Slove Mdeg'
  // SampleTimeMath: '<S6>/TSamp' incorporates:
  //   Gain: '<S1>/sailD'
  //
  //  About '<S6>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Oyaw = rtP.sailD * rtB.sf_inftopi.a * rtP.TSamp_WtEt;

  // Switch: '<S1>/Switch1' incorporates:
  //   Abs: '<S1>/Abs1'
  //   Constant: '<S1>/Constant2'
  //   Constant: '<S1>/Constant3'
  //   Inport: '<Root>/roll'

  if (std::abs(rtU.roll) > rtP.Switch1_Threshold) {
    rtb_Saturation2 = rtP.Constant2_Value;
  } else {
    rtb_Saturation2 = rtP.Constant3_Value;
  }

  // End of Switch: '<S1>/Switch1'

  // Sum: '<S1>/Subtract1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   Gain: '<S1>/sailP'
  //   Memory: '<S1>/Memory'
  //   Product: '<S1>/Product1'
  //   Sum: '<S1>/Add1'
  //   Sum: '<S6>/Diff'
  //   UnitDelay: '<S6>/UD'
  //
  //  Block description for '<S6>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S6>/UD':
  //
  //   Store in Global RAM

  rtb_Saturation2 = rtDW.Memory_PreviousInput - ((rtP.sailP * rtB.sf_inftopi.a +
    rtDW.DiscreteTimeIntegrator_DSTATE_j) + (rtb_Oyaw - rtDW.UD_DSTATE_o)) *
    rtb_Saturation2;

  // Saturate: '<S1>/Saturation1'
  if (rtb_Saturation2 > rtP.Saturation1_UpperSat) {
    rtb_Saturation2 = rtP.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation2 < rtP.Saturation1_LowerSat) {
      rtb_Saturation2 = rtP.Saturation1_LowerSat;
    }
  }

  // Sum: '<S1>/Add2' incorporates:
  //   Memory: '<S1>/Memory'
  //   Saturate: '<S1>/Saturation1'

  rtb_TSamp = rtDW.Memory_PreviousInput - rtb_Saturation2;

  // Saturate: '<S1>/Saturation3'
  if (rtb_TSamp > rtP.Saturation3_UpperSat) {
    rtb_TSamp = rtP.Saturation3_UpperSat;
  } else {
    if (rtb_TSamp < rtP.Saturation3_LowerSat) {
      rtb_TSamp = rtP.Saturation3_LowerSat;
    }
  }

  // End of Saturate: '<S1>/Saturation3'

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S1>/sailI'

  rtDW.DiscreteTimeIntegrator_DSTATE_j += rtP.sailI * rtB.sf_inftopi.a *
    rtP.DiscreteTimeIntegrator_gainval;

  // Update for UnitDelay: '<S6>/UD'
  //
  //  Block description for '<S6>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_o = rtb_Oyaw;

  // Update for Memory: '<S1>/Memory'
  rtDW.Memory_PreviousInput = rtb_TSamp;

  // End of Outputs for SubSystem: '<Root>/ Slove Mdeg'

  // Outport: '<Root>/Sail'
  rtY.Sail = rtb_TSamp;
}

// Model initialize function
void sailboat_ap_okModelClass::initialize()
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus((&rtM), (NULL));

  // block I/O
  (void) memset(((void *) &rtB), 0,
                sizeof(B));

  // states (dwork)
  (void) memset((void *)&rtDW, 0,
                sizeof(DW));

  // external inputs
  (void) memset((void *)&rtU, 0,
                sizeof(ExtU));

  // external outputs
  (void) memset((void *)&rtY, 0,
                sizeof(ExtY));

  // InitializeConditions for Atomic SubSystem: '<Root>/slove Rdeg'
  // InitializeConditions for Memory: '<S5>/Memory1'
  rtDW.Memory1_PreviousInput = rtP.Memory1_X0;

  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_DSTATE = rtP.DiscreteTimeIntegrator_IC_e;

  // InitializeConditions for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE = rtP.DiscreteDerivative_ICPrevScal_g;

  // End of InitializeConditions for SubSystem: '<Root>/slove Rdeg'

  // InitializeConditions for Atomic SubSystem: '<Root>/ Slove Mdeg'
  // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_DSTATE_j = rtP.DiscreteTimeIntegrator_IC;

  // InitializeConditions for UnitDelay: '<S6>/UD'
  //
  //  Block description for '<S6>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_o = rtP.DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for Memory: '<S1>/Memory'
  rtDW.Memory_PreviousInput = rtP.Memory_X0;

  // End of InitializeConditions for SubSystem: '<Root>/ Slove Mdeg'
}

// Model terminate function
void sailboat_ap_okModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
sailboat_ap_okModelClass::sailboat_ap_okModelClass()
{
  P rtP_temp = {
    0.0,                               // Variable: oyaw
                                       //  Referenced by: '<Root>/Oyaw'

    0.05,                              // Variable: rudderD
                                       //  Referenced by: '<S5>/rudderD'

    0.0,                               // Variable: rudderI
                                       //  Referenced by: '<S5>/rudderI'

    0.6,                               // Variable: rudderP
                                       //  Referenced by: '<S5>/rudderP'

    0.0,                               // Variable: sailD
                                       //  Referenced by: '<S1>/sailD'

    0.0,                               // Variable: sailI
                                       //  Referenced by: '<S1>/sailI'

    -0.44,                             // Variable: sailP
                                       //  Referenced by: '<S1>/sailP'

    0.0,                               // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                       //  Referenced by: '<S6>/UD'

    0.0,                               // Mask Parameter: DiscreteDerivative_ICPrevScal_g
                                       //  Referenced by: '<S7>/UD'

    1.0,                               // Expression: 1
                                       //  Referenced by: '<S1>/Constant3'

    1.2,                               // Expression: 1.2
                                       //  Referenced by: '<S1>/Constant2'

    0.1,                               // Computed Parameter: DiscreteTimeIntegrator_gainval
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

    10.0,                              // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S6>/TSamp'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<S1>/Memory'

    0.25,                              // Expression: 0.25
                                       //  Referenced by: '<S1>/Switch1'

    0.2,                               // Expression: 0.20
                                       //  Referenced by: '<S1>/Saturation1'

    -0.2,                              // Expression: -0.20
                                       //  Referenced by: '<S1>/Saturation1'

    1.39,                              // Expression: 1.39
                                       //  Referenced by: '<S1>/Saturation3'

    -1.39,                             // Expression: -1.39
                                       //  Referenced by: '<S1>/Saturation3'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<S5>/Memory1'

    0.1,                               // Computed Parameter: DiscreteTimeIntegrator_gainva_h
                                       //  Referenced by: '<S5>/Discrete-Time Integrator'

    0.0,                               // Expression: 0
                                       //  Referenced by: '<S5>/Discrete-Time Integrator'

    10.0,                              // Computed Parameter: TSamp_WtEt_i
                                       //  Referenced by: '<S7>/TSamp'

    0.175,                             // Expression: 0.175
                                       //  Referenced by: '<S5>/Saturation'

    -0.175,                            // Expression: -0.175
                                       //  Referenced by: '<S5>/Saturation'

    0.68,                              // Expression: 0.68
                                       //  Referenced by: '<S5>/Saturation2'

    -0.68,                             // Expression: -0.68
                                       //  Referenced by: '<S5>/Saturation2'

    0.68,                              // Expression: 0.68
                                       //  Referenced by: '<Root>/Saturation4'

    -0.68                              // Expression: -0.68
                                       //  Referenced by: '<Root>/Saturation4'

  };                                   // Modifiable parameters

  // Initialize tunable parameters
  rtP = rtP_temp;
}

// Destructor
sailboat_ap_okModelClass::~sailboat_ap_okModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * sailboat_ap_okModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//

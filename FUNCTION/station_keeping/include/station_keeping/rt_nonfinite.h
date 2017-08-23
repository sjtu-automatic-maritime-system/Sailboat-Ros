//
// File: rt_nonfinite.h
//
// Code generated for Simulink model 'keeping'.
//
// Model version                  : 1.182
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Wed Aug 23 15:56:21 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//

#ifndef RTW_HEADER_rt_nonfinite_h_
#define RTW_HEADER_rt_nonfinite_h_
#include <stddef.h>
#include "rtwtypes.h"
#if defined(_MSC_VER) && (_MSC_VER <= 1200)

// For MSVC 6.0, use a compiler specific comparison function
#include <float.h>
#endif

#ifdef __cplusplus

extern "C" {

#endif

  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  extern void rt_InitInfAndNaN(size_t realSize);
  extern boolean_T rtIsInf(real_T value);
  extern boolean_T rtIsInfF(real32_T value);
  extern boolean_T rtIsNaN(real_T value);
  extern boolean_T rtIsNaNF(real32_T value);
  typedef struct {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  } BigEndianIEEEDouble;

  typedef struct {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  } LittleEndianIEEEDouble;

  typedef struct {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  } IEEESingle;

#ifdef __cplusplus

}                                      // extern "C"
#endif
#endif                                 // RTW_HEADER_rt_nonfinite_h_

//
// File trailer for generated code.
//
// [EOF]
//

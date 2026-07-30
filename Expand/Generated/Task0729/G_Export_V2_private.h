/*
 * File: G_Export_V2_private.h
 *
 * Code generated for Simulink model 'G_Export_V2'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Thu Jul 30 10:48:36 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef G_Export_V2_private_h_
#define G_Export_V2_private_h_
#include "rtwtypes.h"
#include "G_Export_V2_types.h"

extern void MWDSPCG_FFT_Interleave_R2BR_S(const real32_T x[], creal32_T y[],
  int32_T nChans, int32_T nRows);
extern void MWDSPCG_R2DIT_TBLS_C(creal32_T y[], int32_T nChans, int32_T nRows,
  int32_T fftLen, int32_T offset, const real32_T tablePtr[], int32_T twiddleStep,
  boolean_T isInverse);
extern void MWDSPCG_FFT_DblLen_C(creal32_T y[], int32_T nChans, int32_T nRows,
  const real32_T twiddleTable[], int32_T twiddleStep);
extern real32_T rt_hypotf(real32_T u0, real32_T u1);
extern real_T rt_roundd(real_T u);

#endif                                 /* G_Export_V2_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

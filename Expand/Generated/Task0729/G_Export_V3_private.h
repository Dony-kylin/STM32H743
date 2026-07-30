#ifndef G_Export_V3_private_h_
#define G_Export_V3_private_h_
#include "rtwtypes.h"
#include "G_Export_V3_types.h"

extern void MWDSPCG_FFT_Interleave_R2BR_S(const real32_T x[], creal32_T y[],
  int32_T nChans, int32_T nRows);
extern void MWDSPCG_R2DIT_TBLS_C(creal32_T y[], int32_T nChans, int32_T nRows,
  int32_T fftLen, int32_T offset, const real32_T tablePtr[], int32_T twiddleStep,
  boolean_T isInverse);
extern void MWDSPCG_FFT_DblLen_C(creal32_T y[], int32_T nChans, int32_T nRows,
  const real32_T twiddleTable[], int32_T twiddleStep);
extern real32_T rt_hypotf(real32_T u0, real32_T u1);

#endif


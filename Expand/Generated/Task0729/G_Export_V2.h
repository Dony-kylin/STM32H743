/*
 * File: G_Export_V2.h
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

#ifndef G_Export_V2_h_
#define G_Export_V2_h_
#ifndef G_Export_V2_COMMON_INCLUDES_
#define G_Export_V2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* G_Export_V2_COMMON_INCLUDES_ */

#include "G_Export_V2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T Q15ADC[16384];              /* '<Root>/Q15转ADC电压' */
  creal32_T u096FFT[4096];             /* '<Root>/4096点FFT' */
  real32_T FIR4[4096];                 /* '<Root>/FIR低通并4倍抽取' */
  real32_T Hann[4096];                 /* '<Root>/Hann窗' */
} B_G_Export_V2_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T FIR4_Sums[16];              /* '<Root>/FIR低通并4倍抽取' */
  real32_T FIR4_StatesBuff[15];        /* '<Root>/FIR低通并4倍抽取' */
  int32_T FIR4_PhaseIdx;               /* '<Root>/FIR低通并4倍抽取' */
} DW_G_Export_V2_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: FIR4_FILT
   * Referenced by: '<Root>/FIR低通并4倍抽取'
   */
  real32_T FIR4_FILT[64];

  /* Computed Parameter: Hann_WindowSamples
   * Referenced by: '<Root>/Hann窗'
   */
  real32_T Hann_WindowSamples[4096];

  /* Computed Parameter: u096FFT_TwiddleTable
   * Referenced by: '<Root>/4096点FFT'
   */
  real32_T u096FFT_TwiddleTable[3072];
} ConstP_G_Export_V2_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  int16_T adc_block[16384];            /* '<Root>/adc_block' */
  uint8_T mode;                        /* '<Root>/mode' */
  uint8_T periods;                     /* '<Root>/periods' */
} ExtU_G_Export_V2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T frequency_Hz[3];            /* '<Root>/frequency_Hz' */
  real32_T amplitude_Vpk[3];           /* '<Root>/amplitude_Vpk' */
  uint8_T component_count;             /* '<Root>/component_count' */
  uint8_T harmonic_order[3];           /* '<Root>/harmonic_order' */
  real32_T Vpp;                        /* '<Root>/Vpp' */
  real32_T Vrms;                       /* '<Root>/Vrms' */
  real32_T fundamental_Hz;             /* '<Root>/fundamental_Hz' */
  real32_T waveform[600];              /* '<Root>/waveform' */
  uint16_T waveCount;                  /* '<Root>/waveCount' */
} ExtY_G_Export_V2_T;

/* Real-time Model Data Structure */
struct tag_RTM_G_Export_V2_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_G_Export_V2_T G_Export_V2_B;

/* Block states (default storage) */
extern DW_G_Export_V2_T G_Export_V2_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_G_Export_V2_T G_Export_V2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_G_Export_V2_T G_Export_V2_Y;

/* Constant parameters (default storage) */
extern const ConstP_G_Export_V2_T G_Export_V2_ConstP;

/* Model entry point functions */
extern void G_Export_V2_initialize(void);
extern void G_Export_V2_step(void);
extern void G_Export_V2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_G_Export_V2_T *const G_Export_V2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'G_Export_V2'
 * '<S1>'   : 'G_Export_V2/时域测量'
 * '<S2>'   : 'G_Export_V2/频率幅值提取'
 * '<S3>'   : 'G_Export_V2/时域测量/计算测量值'
 * '<S4>'   : 'G_Export_V2/频率幅值提取/谱峰提取'
 */
#endif                                 /* G_Export_V2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

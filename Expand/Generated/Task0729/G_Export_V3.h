#ifndef G_Export_V3_h_
#define G_Export_V3_h_
#ifndef G_Export_V3_COMMON_INCLUDES_
#define G_Export_V3_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "G_Export_V3_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real32_T Q15ADC[16384];
  creal32_T u096FFT[4096];
  real32_T FIR4[4096];
  real32_T Hann[4096];
} B_G_Export_V3_T;

typedef struct {
  real32_T FIR4_Sums[16];
  real32_T FIR4_StatesBuff[15];
  int32_T FIR4_PhaseIdx;
} DW_G_Export_V3_T;

typedef struct {
  real32_T FIR4_FILT[64];
  real32_T Hann_WindowSamples[4096];
  real32_T u096FFT_TwiddleTable[3072];
} ConstP_G_Export_V3_T;

typedef struct {
  int16_T adc_block[16384];
  uint8_T mode;
  uint8_T periods;
} ExtU_G_Export_V3_T;

typedef struct {
  real32_T frequency_Hz[3];
  real32_T amplitude_Vpk[3];
  uint8_T component_count;
  uint8_T harmonic_order[3];
  real32_T Vpp;
  real32_T Vrms;
  real32_T fundamental_Hz;
  real32_T waveform[600];
  uint16_T waveCount;
  real32_T amplitude_SettingVpk[3];
} ExtY_G_Export_V3_T;

struct tag_RTM_G_Export_V3_T {
  const char_T * volatile errorStatus;
};

extern B_G_Export_V3_T G_Export_V3_B;
extern DW_G_Export_V3_T G_Export_V3_DW;
extern ExtU_G_Export_V3_T G_Export_V3_U;
extern ExtY_G_Export_V3_T G_Export_V3_Y;
extern const ConstP_G_Export_V3_T G_Export_V3_ConstP;
extern void G_Export_V3_initialize(void);
extern void G_Export_V3_step(void);
extern void G_Export_V3_terminate(void);
extern RT_MODEL_G_Export_V3_T *const G_Export_V3_M;

#endif


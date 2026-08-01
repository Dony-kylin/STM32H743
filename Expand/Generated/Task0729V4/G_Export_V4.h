#ifndef G_Export_V4_h_
#define G_Export_V4_h_
#ifndef G_Export_V4_COMMON_INCLUDES_
#define G_Export_V4_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct tag_RTM_G_Export_V4_T RT_MODEL_G_Export_V4_T;
typedef struct {
  creal32_T u096FFT[16384];
  real32_T FIR_[16384];
  real32_T Hann[16384];
  real32_T u[16384];
} B_G_Export_V4_T;

typedef struct {
  real32_T FIR__Sums[64];
  real32_T FIR__StatesBuff[63];
  int32_T FIR__PhaseIdx;
} DW_G_Export_V4_T;

typedef struct {
  real32_T Hann_WindowSamples[16384];
  real32_T u096FFT_TwiddleTable[12288];
} ConstP_G_Export_V4_T;

typedef struct {
  int16_T adc_block[16384];
  uint8_T mode;
} ExtU_G_Export_V4_T;

typedef struct {
  real32_T frequency_Hz[3];
  real32_T amplitude_Vpk[3];
  uint8_T component_count;
  uint8_T harmonic_order[3];
  real32_T Vpp;
  real32_T Vrms;
  real32_T fundamental_Hz;
  real32_T amplitude_SettingVpk[3];
} ExtY_G_Export_V4_T;

struct P_G_Export_V4_T_ {
  real32_T Q15ADC_Gain;
  real32_T FIR__FILT[64];
  real32_T _amplitude_Vpk_Gain;
  real32_T _Vpp_Gain;
  real32_T _Vrms_Gain;
  real32_T _amplitude_SettingVpk_Gain;
  boolean_T frame_valid_Value;
  uint8_T _Threshold;
};

typedef struct P_G_Export_V4_T_ P_G_Export_V4_T;
struct tag_RTM_G_Export_V4_T {
  const char_T * volatile errorStatus;
};

extern P_G_Export_V4_T G_Export_V4_P;
extern B_G_Export_V4_T G_Export_V4_B;
extern DW_G_Export_V4_T G_Export_V4_DW;
extern ExtU_G_Export_V4_T G_Export_V4_U;
extern ExtY_G_Export_V4_T G_Export_V4_Y;
extern const ConstP_G_Export_V4_T G_Export_V4_ConstP;
extern void G_Export_V4_initialize(void);
extern void G_Export_V4_step(void);
extern void G_Export_V4_terminate(void);
extern RT_MODEL_G_Export_V4_T *const G_Export_V4_M;

#endif

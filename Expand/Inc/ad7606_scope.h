#ifndef __AD7606_SCOPE_H__
#define __AD7606_SCOPE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Keep one of every four conversions for the LCD timebase. With a 200 kSPS
 * converter rate, the 4096-point display history spans about 82 ms and still
 * has a 50 kSPS effective display rate. Set to 1 for high-frequency inputs.
 */
#ifndef AD7606_SCOPE_DECIMATION
#define AD7606_SCOPE_DECIMATION 4U
#endif
#if (AD7606_SCOPE_DECIMATION < 1U)
#error "AD7606_SCOPE_DECIMATION must be at least 1"
#endif

typedef struct
{
  uint8_t channel;             /* 1..8 */
  uint8_t running;             /* 1 = running, 0 = hold */
  uint8_t center_auto;         /* 1 = follow measured DC level */
  uint8_t reserved;
  uint16_t mv_per_div;
  uint16_t decimation;
  int32_t center_mv;           /* voltage represented by the center grid line */
  uint32_t time_per_div_us;    /* 0 = automatic timebase */
  uint32_t refresh_ms;
} AD7606_ScopeConfig;

void AD7606_ScopeInit(uint32_t full_scale_mv);
void AD7606_ScopePushFrame(const int16_t *channels);
void AD7606_ScopeDisplayInit(void);
void AD7606_ScopeDisplayRefresh(void);
void AD7606_ScopeGetConfig(AD7606_ScopeConfig *config);
uint8_t AD7606_ScopeSetChannel(uint32_t channel);
uint8_t AD7606_ScopeSetMvPerDiv(uint32_t mv_per_div);
uint8_t AD7606_ScopeSetDecimation(uint32_t decimation);
uint8_t AD7606_ScopeSetCenterMv(int32_t center_mv);
void AD7606_ScopeSetCenterAuto(uint8_t automatic);
uint8_t AD7606_ScopeSetTimePerDivUs(uint32_t time_per_div_us);
uint8_t AD7606_ScopeSetRefreshMs(uint32_t refresh_ms);
void AD7606_ScopeSetRunning(uint8_t running);
uint32_t AD7606_ScopeGetRefreshMs(void);
uint32_t AD7606_ScopeGetInputSampleRateHz(void);
uint8_t AD7606_ScopeAutoConfigure(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD7606_SCOPE_H__ */

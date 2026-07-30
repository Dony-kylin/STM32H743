#ifndef __AD7606_SCOPE_H__
#define __AD7606_SCOPE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Keep every AD9220 sample so 500 kHz still has four points per period. */
#ifndef AD7606_SCOPE_DECIMATION
#define AD7606_SCOPE_DECIMATION 1U
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

typedef struct
{
  int32_t voltage_mv;          /* newest displayed sample */
  int32_t dc_mv;               /* arithmetic mean */
  uint32_t amplitude_mv;       /* half of peak-to-peak voltage */
  uint32_t peak_to_peak_mv;
  uint32_t rms_mv;             /* AC RMS with DC removed */
  uint32_t frequency_millihz;
  uint32_t sample_rate_hz;
  uint8_t valid;
} AD7606_ScopeMeasurements;

void AD7606_ScopeInit(uint32_t full_scale_mv);
void AD7606_ScopePushFrame(const int16_t *channels);
void AD7606_ScopePushSamples(const int16_t *samples, uint32_t count,
                             uint32_t input_sample_rate_hz);
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
uint8_t AD7606_ScopeGetMeasurements(AD7606_ScopeMeasurements *measurements);

#ifdef __cplusplus
}
#endif

#endif /* __AD7606_SCOPE_H__ */

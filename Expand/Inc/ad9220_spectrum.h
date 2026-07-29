#ifndef __AD9220_SPECTRUM_H__
#define __AD9220_SPECTRUM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define AD9220_SPECTRUM_FFT_SIZE          16384U
#define AD9220_SPECTRUM_BIN_COUNT         \
  ((AD9220_SPECTRUM_FFT_SIZE / 2U) + 1U)
#define AD9220_SPECTRUM_HARMONIC_COUNT    10U

typedef struct
{
  uint32_t sequence;
  uint32_t sample_rate_hz;
  uint32_t fft_size;
  uint32_t bin_width_millihz;
  uint32_t bad_sample_count;
  uint32_t analysis_time_us;
  uint32_t fundamental_millihz;
  uint32_t fundamental_amplitude_uv;
  uint32_t thd_ppm;
  uint32_t harmonic_frequency_millihz[
      AD9220_SPECTRUM_HARMONIC_COUNT];
  uint32_t harmonic_amplitude_uv[
      AD9220_SPECTRUM_HARMONIC_COUNT];
  uint8_t valid;
} AD9220_SpectrumResult;

/*
 * Samples at or beyond +/-2.45 V are treated as capture glitches. A complete
 * run of bad samples is replaced by a linear interpolation between the
 * nearest valid samples. At a block edge, the nearest valid sample is held.
 */
uint32_t AD9220_SpectrumRepairSamples(int16_t *samples, uint32_t count);

/*
 * Runs one 16384-point Hann-windowed real FFT and calculates the fundamental,
 * H1..H10 peak amplitudes and THD. Samples are signed Q15 values where
 * +/-32768 corresponds to +/-2.5 V.
 */
uint8_t AD9220_SpectrumAnalyze(const int16_t *samples, uint32_t count,
                               uint32_t sample_rate_hz,
                               uint32_t bad_sample_count,
                               AD9220_SpectrumResult *result);

/* Magnitude of one bin from the most recent completed FFT, in microvolts. */
uint32_t AD9220_SpectrumGetBinMagnitudeUv(uint32_t bin);

/* 0 = idle/complete; nonzero values identify the current analysis stage. */
uint32_t AD9220_SpectrumGetStage(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9220_SPECTRUM_H__ */

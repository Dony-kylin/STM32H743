#include "ad9220_spectrum.h"

#include "stm32h7xx_hal.h"

#include <math.h>
#include <string.h>

#define AD9220_SPECTRUM_PI_F                 3.14159265358979323846f
#define AD9220_SPECTRUM_FULL_SCALE_UV        2500000.0f
#define AD9220_SPECTRUM_BAD_LIMIT_RAW        32113
#define AD9220_SPECTRUM_MIN_FUNDAMENTAL_HZ    1000U
#define AD9220_SPECTRUM_MIN_SIGNAL_UV            1000U
#define AD9220_SPECTRUM_RAM \
  __attribute__((section(".scope_ram"), aligned(32)))

static float spectrum_fft_buffer[2U * AD9220_SPECTRUM_FFT_SIZE]
    AD9220_SPECTRUM_RAM;
static float spectrum_window_sum;
static uint32_t spectrum_sequence;
static uint8_t spectrum_fft_valid;
static volatile uint32_t spectrum_stage;

static uint8_t AD9220_SpectrumIsBad(int16_t sample);
static float AD9220_SpectrumBinPower(uint32_t bin);
static void AD9220_SpectrumRunFft(void);
static float AD9220_SpectrumToneAmplitudeRaw(
    const int16_t *samples, int32_t mean, float frequency_hz,
    uint32_t sample_rate_hz);

__attribute__((optimize("O3")))
uint32_t AD9220_SpectrumRepairSamples(int16_t *samples, uint32_t count)
{
  uint32_t bad_count = 0U;
  uint32_t index = 0U;

  if ((samples == NULL) || (count == 0U))
  {
    return 0U;
  }

  while (index < count)
  {
    uint32_t run_start;
    uint32_t run_end;
    uint8_t have_left;
    uint8_t have_right;
    int32_t left = 0;
    int32_t right = 0;

    if (AD9220_SpectrumIsBad(samples[index]) == 0U)
    {
      ++index;
      continue;
    }

    run_start = index;
    while ((index < count) &&
           (AD9220_SpectrumIsBad(samples[index]) != 0U))
    {
      ++index;
      ++bad_count;
    }
    run_end = index;

    have_left = (run_start != 0U) ? 1U : 0U;
    have_right = (run_end < count) ? 1U : 0U;
    if (have_left != 0U)
    {
      left = samples[run_start - 1U];
    }
    if (have_right != 0U)
    {
      right = samples[run_end];
    }

    if ((have_left != 0U) && (have_right != 0U))
    {
      uint32_t span = run_end - run_start + 1U;

      for (uint32_t position = run_start;
           position < run_end; ++position)
      {
        uint32_t step = position - run_start + 1U;
        int64_t interpolated =
            (int64_t)left +
            (((int64_t)right - left) * step) / span;
        samples[position] = (int16_t)interpolated;
      }
    }
    else
    {
      int16_t replacement = 0;

      if (have_left != 0U)
      {
        replacement = (int16_t)left;
      }
      else if (have_right != 0U)
      {
        replacement = (int16_t)right;
      }

      for (uint32_t position = run_start;
           position < run_end; ++position)
      {
        samples[position] = replacement;
      }
    }
  }

  return bad_count;
}

__attribute__((optimize("O3")))
uint8_t AD9220_SpectrumAnalyze(const int16_t *samples, uint32_t count,
                               uint32_t sample_rate_hz,
                               uint32_t bad_sample_count,
                               AD9220_SpectrumResult *result)
{
  int64_t sum = 0;
  int32_t mean;
  float window_cos = 1.0f;
  float window_sin = 0.0f;
  float window_step_cos;
  float window_step_sin;
  float peak_power = 0.0f;
  uint32_t minimum_bin;
  uint32_t maximum_bin;
  uint32_t peak_bin = 0U;
  float fractional_bin;
  float fundamental_hz;
  float harmonic_square_sum = 0.0f;
  uint32_t start_cycles;
  uint32_t elapsed_cycles;

  if ((samples == NULL) || (result == NULL) ||
      (count != AD9220_SPECTRUM_FFT_SIZE) ||
      (sample_rate_hz == 0U))
  {
    spectrum_stage = 0xE1U;
    return 0U;
  }

  memset(result, 0, sizeof(*result));
  start_cycles = DWT->CYCCNT;
  spectrum_fft_valid = 0U;
  spectrum_stage = 1U;

  for (uint32_t i = 0U; i < count; ++i)
  {
    sum += samples[i];
  }
  mean = (int32_t)(sum / (int64_t)count);

  window_step_cos =
      cosf((2.0f * AD9220_SPECTRUM_PI_F) /
           (float)(AD9220_SPECTRUM_FFT_SIZE - 1U));
  window_step_sin =
      sinf((2.0f * AD9220_SPECTRUM_PI_F) /
           (float)(AD9220_SPECTRUM_FFT_SIZE - 1U));
  spectrum_window_sum = 0.0f;

  for (uint32_t i = 0U; i < AD9220_SPECTRUM_FFT_SIZE; ++i)
  {
    float window = 0.5f - (0.5f * window_cos);
    float next_cos =
        (window_cos * window_step_cos) -
        (window_sin * window_step_sin);
    float next_sin =
        (window_sin * window_step_cos) +
        (window_cos * window_step_sin);

    spectrum_fft_buffer[2U * i] =
        ((float)samples[i] - (float)mean) * window;
    spectrum_fft_buffer[(2U * i) + 1U] = 0.0f;
    spectrum_window_sum += window;
    window_cos = next_cos;
    window_sin = next_sin;
  }

  spectrum_stage = 2U;
  AD9220_SpectrumRunFft();
  spectrum_fft_valid = 1U;
  spectrum_stage = 3U;

  result->sequence = ++spectrum_sequence;
  result->sample_rate_hz = sample_rate_hz;
  result->fft_size = AD9220_SPECTRUM_FFT_SIZE;
  result->bin_width_millihz = (uint32_t)
      ((((uint64_t)sample_rate_hz * 1000U) +
        (AD9220_SPECTRUM_FFT_SIZE / 2U)) /
       AD9220_SPECTRUM_FFT_SIZE);
  result->bad_sample_count = bad_sample_count;

  minimum_bin = (uint32_t)
      ((((uint64_t)AD9220_SPECTRUM_MIN_FUNDAMENTAL_HZ *
         AD9220_SPECTRUM_FFT_SIZE) + sample_rate_hz - 1U) /
       sample_rate_hz);
  maximum_bin = (uint32_t)
      ((AD9220_SPECTRUM_FFT_SIZE / 2U) - 1U);
  if (minimum_bin < 1U)
  {
    minimum_bin = 1U;
  }
  if (maximum_bin >= AD9220_SPECTRUM_BIN_COUNT)
  {
    maximum_bin = AD9220_SPECTRUM_BIN_COUNT - 1U;
  }

  for (uint32_t bin = minimum_bin; bin <= maximum_bin; ++bin)
  {
    float power = AD9220_SpectrumBinPower(bin);

    if (power > peak_power)
    {
      peak_power = power;
      peak_bin = bin;
    }
  }
  if ((peak_bin == 0U) || (peak_power <= 0.0f))
  {
    elapsed_cycles = DWT->CYCCNT - start_cycles;
    result->analysis_time_us = (SystemCoreClock != 0U) ?
        (uint32_t)(((uint64_t)elapsed_cycles * 1000000U) /
                   SystemCoreClock) : 0U;
    spectrum_stage = 0U;
    return 1U;
  }

  fractional_bin = (float)peak_bin;
  if ((peak_bin > minimum_bin) && (peak_bin < maximum_bin))
  {
    float left = AD9220_SpectrumBinPower(peak_bin - 1U);
    float center = peak_power;
    float right = AD9220_SpectrumBinPower(peak_bin + 1U);

    if ((left > 0.0f) && (center > 0.0f) && (right > 0.0f))
    {
      float left_log = logf(left);
      float center_log = logf(center);
      float right_log = logf(right);
      float denominator =
          left_log - (2.0f * center_log) + right_log;

      if (fabsf(denominator) > 1.0e-12f)
      {
        float delta =
            0.5f * (left_log - right_log) / denominator;

        if (delta > 0.5f)
        {
          delta = 0.5f;
        }
        else if (delta < -0.5f)
        {
          delta = -0.5f;
        }
        fractional_bin += delta;
      }
    }
  }

  fundamental_hz =
      fractional_bin * ((float)sample_rate_hz /
                        (float)AD9220_SPECTRUM_FFT_SIZE);

  result->fundamental_millihz =
      (uint32_t)((fundamental_hz * 1000.0f) + 0.5f);

  spectrum_stage = 4U;
  for (uint32_t harmonic = 1U;
       harmonic <= AD9220_SPECTRUM_HARMONIC_COUNT; ++harmonic)
  {
    float harmonic_hz = fundamental_hz * (float)harmonic;

    if (harmonic_hz >= ((float)sample_rate_hz * 0.5f))
    {
      break;
    }

    {
      float amplitude_raw = AD9220_SpectrumToneAmplitudeRaw(
          samples, mean, harmonic_hz, sample_rate_hz);
      float amplitude_uv =
          amplitude_raw *
          (AD9220_SPECTRUM_FULL_SCALE_UV / 32768.0f);
      uint32_t amplitude_uv_rounded =
          (uint32_t)(amplitude_uv + 0.5f);
      uint32_t index = harmonic - 1U;

      result->harmonic_frequency_millihz[index] =
          (uint32_t)((harmonic_hz * 1000.0f) + 0.5f);
      result->harmonic_amplitude_uv[index] =
          amplitude_uv_rounded;
      if (harmonic == 1U)
      {
        result->fundamental_amplitude_uv = amplitude_uv_rounded;
      }
      else
      {
        harmonic_square_sum += amplitude_uv * amplitude_uv;
      }
    }
  }

  if (result->fundamental_amplitude_uv >=
      AD9220_SPECTRUM_MIN_SIGNAL_UV)
  {
    float thd =
        sqrtf(harmonic_square_sum) /
        (float)result->fundamental_amplitude_uv;
    float thd_ppm = thd * 1000000.0f;

    result->thd_ppm = (thd_ppm < 4294967040.0f) ?
        (uint32_t)(thd_ppm + 0.5f) : UINT32_MAX;
  }

  elapsed_cycles = DWT->CYCCNT - start_cycles;
  result->analysis_time_us = (SystemCoreClock != 0U) ?
      (uint32_t)(((uint64_t)elapsed_cycles * 1000000U) /
                 SystemCoreClock) : 0U;
  result->valid =
      (result->fundamental_amplitude_uv >=
       AD9220_SPECTRUM_MIN_SIGNAL_UV) ? 1U : 0U;
  spectrum_stage = 0U;
  return 1U;
}

uint32_t AD9220_SpectrumGetBinMagnitudeUv(uint32_t bin)
{
  float magnitude_raw;
  float magnitude_uv;

  if ((spectrum_fft_valid == 0U) ||
      (spectrum_window_sum <= 0.0f) ||
      (bin >= AD9220_SPECTRUM_BIN_COUNT))
  {
    return 0U;
  }

  if (bin == 0U)
  {
    magnitude_raw = fabsf(spectrum_fft_buffer[0]) /
                    spectrum_window_sum;
  }
  else if (bin == (AD9220_SPECTRUM_FFT_SIZE / 2U))
  {
    magnitude_raw =
        fabsf(spectrum_fft_buffer[AD9220_SPECTRUM_FFT_SIZE]) /
                    spectrum_window_sum;
  }
  else
  {
    magnitude_raw =
        (2.0f * sqrtf(AD9220_SpectrumBinPower(bin))) /
        spectrum_window_sum;
  }
  magnitude_uv =
      magnitude_raw *
      (AD9220_SPECTRUM_FULL_SCALE_UV / 32768.0f);
  return (uint32_t)(magnitude_uv + 0.5f);
}

uint32_t AD9220_SpectrumGetStage(void)
{
  return spectrum_stage;
}

static uint8_t AD9220_SpectrumIsBad(int16_t sample)
{
  int32_t value = sample;

  return ((value >= AD9220_SPECTRUM_BAD_LIMIT_RAW) ||
          (value <= -AD9220_SPECTRUM_BAD_LIMIT_RAW)) ? 1U : 0U;
}

static float AD9220_SpectrumBinPower(uint32_t bin)
{
  float real;
  float imaginary;

  if (bin == 0U)
  {
    real = spectrum_fft_buffer[0];
    return real * real;
  }
  if (bin == (AD9220_SPECTRUM_FFT_SIZE / 2U))
  {
    real = spectrum_fft_buffer[AD9220_SPECTRUM_FFT_SIZE];
    return real * real;
  }
  if (bin >= AD9220_SPECTRUM_BIN_COUNT)
  {
    return 0.0f;
  }

  real = spectrum_fft_buffer[2U * bin];
  imaginary = spectrum_fft_buffer[(2U * bin) + 1U];
  return (real * real) + (imaginary * imaginary);
}

__attribute__((optimize("O3")))
static void AD9220_SpectrumRunFft(void)
{
  uint32_t reversed = 0U;
  uint32_t fft_stage = 0U;

  /* Iterative radix-2 bit reversal. */
  for (uint32_t i = 1U; i < AD9220_SPECTRUM_FFT_SIZE; ++i)
  {
    uint32_t bit = AD9220_SPECTRUM_FFT_SIZE >> 1U;

    while ((reversed & bit) != 0U)
    {
      reversed ^= bit;
      bit >>= 1U;
    }
    reversed ^= bit;

    if (i < reversed)
    {
      float real = spectrum_fft_buffer[2U * i];
      float imaginary = spectrum_fft_buffer[(2U * i) + 1U];

      spectrum_fft_buffer[2U * i] =
          spectrum_fft_buffer[2U * reversed];
      spectrum_fft_buffer[(2U * i) + 1U] =
          spectrum_fft_buffer[(2U * reversed) + 1U];
      spectrum_fft_buffer[2U * reversed] = real;
      spectrum_fft_buffer[(2U * reversed) + 1U] = imaginary;
    }
  }

  /* In-place radix-2 Cooley-Tukey complex FFT. */
  for (uint32_t length = 2U;
       length <= AD9220_SPECTRUM_FFT_SIZE;
       length <<= 1U)
  {
    uint32_t half = length >> 1U;
    float angle = (-2.0f * AD9220_SPECTRUM_PI_F) /
                  (float)length;
    float step_real = cosf(angle);
    float step_imaginary = sinf(angle);

    spectrum_stage = 10U + fft_stage;
    ++fft_stage;

    for (uint32_t base = 0U;
         base < AD9220_SPECTRUM_FFT_SIZE; base += length)
    {
      float twiddle_real = 1.0f;
      float twiddle_imaginary = 0.0f;

      for (uint32_t j = 0U; j < half; ++j)
      {
        uint32_t even = base + j;
        uint32_t odd = even + half;
        float odd_real = spectrum_fft_buffer[2U * odd];
        float odd_imaginary =
            spectrum_fft_buffer[(2U * odd) + 1U];
        float rotated_real =
            (twiddle_real * odd_real) -
            (twiddle_imaginary * odd_imaginary);
        float rotated_imaginary =
            (twiddle_real * odd_imaginary) +
            (twiddle_imaginary * odd_real);
        float even_real = spectrum_fft_buffer[2U * even];
        float even_imaginary =
            spectrum_fft_buffer[(2U * even) + 1U];
        float next_twiddle_real =
            (twiddle_real * step_real) -
            (twiddle_imaginary * step_imaginary);

        spectrum_fft_buffer[2U * even] =
            even_real + rotated_real;
        spectrum_fft_buffer[(2U * even) + 1U] =
            even_imaginary + rotated_imaginary;
        spectrum_fft_buffer[2U * odd] =
            even_real - rotated_real;
        spectrum_fft_buffer[(2U * odd) + 1U] =
            even_imaginary - rotated_imaginary;

        twiddle_imaginary =
            (twiddle_imaginary * step_real) +
            (twiddle_real * step_imaginary);
        twiddle_real = next_twiddle_real;
      }
    }
  }
}

__attribute__((optimize("O3")))
static float AD9220_SpectrumToneAmplitudeRaw(
    const int16_t *samples, int32_t mean, float frequency_hz,
    uint32_t sample_rate_hz)
{
  float phase_cos = 1.0f;
  float phase_sin = 0.0f;
  float phase_step =
      (2.0f * AD9220_SPECTRUM_PI_F * frequency_hz) /
      (float)sample_rate_hz;
  float step_cos = cosf(phase_step);
  float step_sin = sinf(phase_step);
  float window_cos = 1.0f;
  float window_sin = 0.0f;
  float window_step_cos =
      cosf((2.0f * AD9220_SPECTRUM_PI_F) /
           (float)(AD9220_SPECTRUM_FFT_SIZE - 1U));
  float window_step_sin =
      sinf((2.0f * AD9220_SPECTRUM_PI_F) /
           (float)(AD9220_SPECTRUM_FFT_SIZE - 1U));
  float real_sum = 0.0f;
  float imaginary_sum = 0.0f;

  for (uint32_t i = 0U; i < AD9220_SPECTRUM_FFT_SIZE; ++i)
  {
    float window = 0.5f - (0.5f * window_cos);
    float value = ((float)samples[i] - (float)mean) * window;
    float next_phase_cos =
        (phase_cos * step_cos) - (phase_sin * step_sin);
    float next_phase_sin =
        (phase_sin * step_cos) + (phase_cos * step_sin);
    float next_window_cos =
        (window_cos * window_step_cos) -
        (window_sin * window_step_sin);
    float next_window_sin =
        (window_sin * window_step_cos) +
        (window_cos * window_step_sin);

    real_sum += value * phase_cos;
    imaginary_sum -= value * phase_sin;
    phase_cos = next_phase_cos;
    phase_sin = next_phase_sin;
    window_cos = next_window_cos;
    window_sin = next_window_sin;
  }

  return (2.0f * sqrtf((real_sum * real_sum) +
                       (imaginary_sum * imaginary_sum))) /
         spectrum_window_sum;
}

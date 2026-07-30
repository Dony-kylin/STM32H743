#include "task0729_processor.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI 3.14159265358979323846

static int16_t samples[TASK0729_INPUT_SAMPLES];

static int16_t QuantizeAd9220(double input_voltage)
{
  double adc_voltage = input_voltage * TASK0729_FRONTEND_GAIN;
  long raw = lround((adc_voltage / 5.0) * 4096.0 + 2048.0);

  if (raw < 0L)
  {
    raw = 0L;
  }
  else if (raw > 4095L)
  {
    raw = 4095L;
  }
  return (int16_t)((raw - 2048L) << 4);
}

int main(void)
{
  const double f1 = 20000.0;
  const double f2 = 40000.0;
  const double a1 = 0.006400;
  const double a2 = 0.006350;
  const double dc = 0.100000;
  Task0729_Result result;
  uint32_t index;

  for (index = 0U; index < TASK0729_INPUT_SAMPLES; ++index)
  {
    double time = (double)index / TASK0729_INPUT_SAMPLE_RATE_HZ;
    double input =
        dc +
        a1 * sin(2.0 * TEST_PI * f1 * time + 0.2) +
        a2 * sin(2.0 * TEST_PI * f2 * time - 0.7);
    samples[index] = QuantizeAd9220(input);
  }

  Task0729_Init();
  if (Task0729_Process(samples, TASK0729_MODE_QUESTION_3,
                       1U, &result) == 0U)
  {
    return EXIT_FAILURE;
  }

  printf("count=%u f0=%.3f A1=%.6f Vpp=%.6f Vrms=%.6f\n",
         result.component_count, result.fundamental_hz,
         result.amplitude_vpk[0], result.vpp, result.vrms);
  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    if (result.frequency_hz[index] > 0.0F)
    {
      printf("H%u %.3fHz %.6fVpk\n",
             result.harmonic_order[index],
             result.frequency_hz[index],
             result.amplitude_vpk[index]);
    }
  }

  if ((fabs(result.frequency_hz[0] - f1) > 20.0) ||
      (fabs(result.frequency_hz[1] - f2) > 40.0) ||
      (fabs(result.amplitude_vpk[0] - a1) > 0.0004) ||
      (fabs(result.amplitude_vpk[1] - a2) > 0.0004))
  {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

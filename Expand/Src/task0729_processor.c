#include "task0729_processor.h"

#include "G_Export_V3.h"

#include <math.h>
#include <string.h>

static Task0729_Result task0729_last_result;
static uint8_t task0729_history_valid;

static float Task0729_SmoothStableValue(float previous, float current)
{
  float reference = fabsf(previous);
  float delta = fabsf(current - previous);

  if (reference < 1.0E-6F)
  {
    return current;
  }

  /*
   * Small frame-to-frame changes are normally ADC noise or FFT-bin jitter.
   * A change above 5 percent is treated as a real signal change and follows
   * immediately; otherwise use a four-frame exponential average.
   */
  if (delta > reference * 0.05F)
  {
    return current;
  }

  return previous + (current - previous) * 0.25F;
}

void Task0729_Init(void)
{
  memset(&task0729_last_result, 0, sizeof(task0729_last_result));
  task0729_history_valid = 0U;
  G_Export_V3_initialize();
}

uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    uint8_t periods,
    Task0729_Result *result)
{
  uint32_t index;
  uint8_t same_layout;

  if ((samples == NULL) || (result == NULL))
  {
    return 0U;
  }
  if ((mode < TASK0729_MODE_QUESTION_1) ||
      (mode > TASK0729_MODE_QUESTION_3))
  {
    return 0U;
  }
  if ((periods != 1U) && (periods != 3U))
  {
    return 0U;
  }

  memcpy(G_Export_V3_U.adc_block, samples,
         sizeof(G_Export_V3_U.adc_block));
  G_Export_V3_U.mode = (uint8_T)mode;
  G_Export_V3_U.periods = periods;
  G_Export_V3_step();

  same_layout =
      ((task0729_history_valid != 0U) &&
       (task0729_last_result.component_count ==
        G_Export_V3_Y.component_count)) ? 1U : 0U;
  if (same_layout != 0U)
  {
    for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
    {
      if (task0729_last_result.harmonic_order[index] !=
          G_Export_V3_Y.harmonic_order[index])
      {
        same_layout = 0U;
        break;
      }
    }
  }

  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    float frequency = G_Export_V3_Y.frequency_Hz[index];
    float amplitude = G_Export_V3_Y.amplitude_Vpk[index];
    float setting_amplitude =
        G_Export_V3_Y.amplitude_SettingVpk[index];

    if (same_layout != 0U)
    {
      frequency = Task0729_SmoothStableValue(
          task0729_last_result.frequency_hz[index], frequency);
      amplitude = Task0729_SmoothStableValue(
          task0729_last_result.amplitude_vpk[index], amplitude);
      setting_amplitude = Task0729_SmoothStableValue(
          task0729_last_result.amplitude_setting_vpk[index],
          setting_amplitude);
    }

    task0729_last_result.frequency_hz[index] = frequency;
    task0729_last_result.amplitude_vpk[index] = amplitude;
    task0729_last_result.amplitude_setting_vpk[index] =
        setting_amplitude;
    task0729_last_result.harmonic_order[index] =
        G_Export_V3_Y.harmonic_order[index];
  }
  task0729_last_result.component_count =
      G_Export_V3_Y.component_count;
  if (same_layout != 0U)
  {
    task0729_last_result.vpp = Task0729_SmoothStableValue(
        task0729_last_result.vpp, G_Export_V3_Y.Vpp);
    task0729_last_result.vrms = Task0729_SmoothStableValue(
        task0729_last_result.vrms, G_Export_V3_Y.Vrms);
    task0729_last_result.fundamental_hz =
        Task0729_SmoothStableValue(
            task0729_last_result.fundamental_hz,
            G_Export_V3_Y.fundamental_Hz);
  }
  else
  {
    task0729_last_result.vpp = G_Export_V3_Y.Vpp;
    task0729_last_result.vrms = G_Export_V3_Y.Vrms;
    task0729_last_result.fundamental_hz =
        G_Export_V3_Y.fundamental_Hz;
  }
  task0729_last_result.waveform_count =
      G_Export_V3_Y.waveCount;
  memcpy(task0729_last_result.waveform,
         G_Export_V3_Y.waveform,
         sizeof(task0729_last_result.waveform));

  task0729_history_valid = 1U;
  *result = task0729_last_result;
  return 1U;
}

const Task0729_Result *Task0729_GetLastResult(void)
{
  return &task0729_last_result;
}

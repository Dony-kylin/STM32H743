#include "task0729_processor.h"

#include "G_Export_V2.h"

#include <string.h>

static Task0729_Result task0729_last_result;

void Task0729_Init(void)
{
  memset(&task0729_last_result, 0, sizeof(task0729_last_result));
  G_Export_V2_initialize();
}

uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    uint8_t periods,
    Task0729_Result *result)
{
  uint32_t index;

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

  memcpy(G_Export_V2_U.adc_block, samples,
         sizeof(G_Export_V2_U.adc_block));
  G_Export_V2_U.mode = (uint8_T)mode;
  G_Export_V2_U.periods = periods;
  G_Export_V2_step();

  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    task0729_last_result.frequency_hz[index] =
        G_Export_V2_Y.frequency_Hz[index];
    task0729_last_result.amplitude_vpk[index] =
        G_Export_V2_Y.amplitude_Vpk[index];
    task0729_last_result.harmonic_order[index] =
        G_Export_V2_Y.harmonic_order[index];
  }
  task0729_last_result.component_count =
      G_Export_V2_Y.component_count;
  task0729_last_result.vpp = G_Export_V2_Y.Vpp;
  task0729_last_result.vrms = G_Export_V2_Y.Vrms;
  task0729_last_result.fundamental_hz =
      G_Export_V2_Y.fundamental_Hz;
  task0729_last_result.waveform_count =
      G_Export_V2_Y.waveCount;
  memcpy(task0729_last_result.waveform,
         G_Export_V2_Y.waveform,
         sizeof(task0729_last_result.waveform));

  *result = task0729_last_result;
  return 1U;
}

const Task0729_Result *Task0729_GetLastResult(void)
{
  return &task0729_last_result;
}

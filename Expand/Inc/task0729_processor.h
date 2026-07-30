#ifndef TASK0729_PROCESSOR_H
#define TASK0729_PROCESSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TASK0729_INPUT_SAMPLES       16384U
#define TASK0729_COMPONENT_COUNT     3U
#define TASK0729_WAVEFORM_SAMPLES    600U

typedef enum
{
  TASK0729_MODE_QUESTION_1 = 1U,
  TASK0729_MODE_QUESTION_2 = 2U,
  TASK0729_MODE_QUESTION_3 = 3U
} Task0729_Mode;

typedef struct
{
  float frequency_hz[TASK0729_COMPONENT_COUNT];
  float amplitude_vpk[TASK0729_COMPONENT_COUNT];
  uint8_t harmonic_order[TASK0729_COMPONENT_COUNT];
  uint8_t component_count;
  float vpp;
  float vrms;
  float fundamental_hz;
  float waveform[TASK0729_WAVEFORM_SAMPLES];
  uint16_t waveform_count;
} Task0729_Result;

/*
 * Initializes the generated Simulink processor. Call once after reset.
 */
void Task0729_Init(void);

/*
 * Processes one complete 8 MHz AD9220 block.
 *
 * samples:
 *   16384 signed, midpoint-removed ADC samples. +/-32768 represents
 *   +/-2.5 V at the ADC input. The generated model compensates the
 *   analog front-end gain of 4 in its final voltage outputs.
 *
 * mode:
 *   TASK0729_MODE_QUESTION_1, _2, or _3.
 *
 * periods:
 *   Selects 1 or 3 complete periods. The selected interval is resampled
 *   after frame processing to exactly TASK0729_WAVEFORM_SAMPLES points.
 *
 * Returns 1 on success, 0 for an invalid argument.
 */
uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    uint8_t periods,
    Task0729_Result *result);

/*
 * Returns the most recent completed result owned by this module.
 */
const Task0729_Result *Task0729_GetLastResult(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK0729_PROCESSOR_H */

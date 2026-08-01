#ifndef TASK0729_PROCESSOR_H
#define TASK0729_PROCESSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TASK0729_INPUT_SAMPLES       16384U
#define TASK0729_COMPONENT_COUNT     3U
#define TASK0729_INPUT_SAMPLE_RATE_HZ 8000000.0F
#define TASK0729_ADC_FULL_SCALE_VPK   2.5F

/*
 * Voltage scaling is kept outside the generated Simulink sources so code
 * regeneration cannot overwrite the hardware calibration.
 *
 * FRONTEND_GAIN is the measured analog gain from the signal input to the
 * AD9220 input. VOLTAGE_CALIBRATION is a final dimensionless trim factor.
 */
#ifndef TASK0729_FRONTEND_GAIN
#define TASK0729_FRONTEND_GAIN        4.08F
#endif

#ifndef TASK0729_VOLTAGE_CALIBRATION
#define TASK0729_VOLTAGE_CALIBRATION  1.0F
#endif

typedef enum
{
  TASK0729_MODE_QUESTION_1 = 1U,
  TASK0729_MODE_QUESTION_2 = 2U,
  TASK0729_MODE_QUESTION_3 = 3U
} Task0729_Mode;

typedef struct
{
  float frequency_hz[TASK0729_COMPONENT_COUNT];
  /* Physical component amplitudes actually present at the ADC input. */
  float amplitude_vpk[TASK0729_COMPONENT_COUNT];
  /* Harmonic-generator setting amplitudes recovered from peak normalization. */
  float amplitude_setting_vpk[TASK0729_COMPONENT_COUNT];
  uint8_t harmonic_order[TASK0729_COMPONENT_COUNT];
  uint8_t component_count;
  float vpp;
  float vrms;
  float fundamental_hz;
} Task0729_Result;

/*
 * Initializes the generated Simulink processor. Call once after reset.
 */
void Task0729_Init(void);

/*
 * Selects the final generator calibration fitted from the measured table.
 * 0: keep raw measured values; nonzero: output fitted values.
 * This selection is independent of Task0729_Process().
 */
void Task0729_SetGeneratorCorrection(uint8_t enable);
uint8_t Task0729_GetGeneratorCorrection(void);

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
 * Returns 1 on success, 0 for an invalid argument.
 */
uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    Task0729_Result *result);

/*
 * Converts one signed Q15 ADC sample to the calibrated external-input
 * voltage. This uses the same scale as amplitude_vpk, vpp and vrms.
 */
float Task0729_SampleToInputVolts(int16_t sample);

/*
 * Returns the most recent completed result owned by this module.
 */
const Task0729_Result *Task0729_GetLastResult(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK0729_PROCESSOR_H */

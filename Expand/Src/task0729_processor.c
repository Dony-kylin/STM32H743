#include "task0729_processor.h"

#include "G_Export_V2.h"

#include <math.h>
#include <string.h>

#define TASK0729_FIT_MAX_TERMS \
  (1U + (2U * TASK0729_COMPONENT_COUNT))
#define TASK0729_GENERATED_FRONTEND_GAIN 4.0F
#define TASK0729_TWO_PI 6.28318530717958647692
#define TASK0729_FIT_PIVOT_EPSILON 1.0E-8

typedef struct
{
  double normal[TASK0729_FIT_MAX_TERMS][TASK0729_FIT_MAX_TERMS];
  double rhs[TASK0729_FIT_MAX_TERMS];
  double solution[TASK0729_FIT_MAX_TERMS];
  double basis[TASK0729_FIT_MAX_TERMS];
  double cosine[TASK0729_COMPONENT_COUNT];
  double sine[TASK0729_COMPONENT_COUNT];
  double cosine_step[TASK0729_COMPONENT_COUNT];
  double sine_step[TASK0729_COMPONENT_COUNT];
  uint8_t result_index[TASK0729_COMPONENT_COUNT];
} Task0729_FitWorkspace;

static Task0729_Result task0729_last_result;
static Task0729_FitWorkspace task0729_fit;

static float Task0729_OutputScaleCorrection(void);
static void Task0729_ResetOscillators(uint32_t component_count);
static void Task0729_AdvanceOscillators(
    uint32_t component_count, uint32_t sample_index);
static uint8_t Task0729_SolveNormalEquations(uint32_t dimension);
static uint8_t Task0729_RefineAmplitudes(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Result *result);

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

  {
    float output_scale = Task0729_OutputScaleCorrection();

    for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
    {
      task0729_last_result.frequency_hz[index] =
          G_Export_V2_Y.frequency_Hz[index];
      task0729_last_result.amplitude_vpk[index] =
          G_Export_V2_Y.amplitude_Vpk[index] * output_scale;
      task0729_last_result.harmonic_order[index] =
          G_Export_V2_Y.harmonic_order[index];
    }
    task0729_last_result.component_count =
        G_Export_V2_Y.component_count;
    task0729_last_result.vpp = G_Export_V2_Y.Vpp * output_scale;
    task0729_last_result.vrms = G_Export_V2_Y.Vrms * output_scale;
    task0729_last_result.fundamental_hz =
        G_Export_V2_Y.fundamental_Hz;
    task0729_last_result.waveform_count =
        G_Export_V2_Y.waveCount;
    for (index = 0U; index < TASK0729_WAVEFORM_SAMPLES; ++index)
    {
      task0729_last_result.waveform[index] =
          G_Export_V2_Y.waveform[index] * output_scale;
    }
  }

  /*
   * The generated model estimates each tone independently. Refit every
   * detected component together against the original 8 MSPS block, including
   * a DC term. Joint least squares prevents one strong harmonic or a DC
   * offset from biasing the amplitude of another component.
   */
  (void)Task0729_RefineAmplitudes(samples, &task0729_last_result);

  *result = task0729_last_result;
  return 1U;
}

float Task0729_SampleToInputVolts(int16_t sample)
{
  float gain = TASK0729_FRONTEND_GAIN;

  if (!(gain > 0.0F))
  {
    gain = TASK0729_GENERATED_FRONTEND_GAIN;
  }

  return ((float)sample *
          (TASK0729_ADC_FULL_SCALE_VPK / 32768.0F) /
          gain) * TASK0729_VOLTAGE_CALIBRATION;
}

static float Task0729_OutputScaleCorrection(void)
{
  float gain = TASK0729_FRONTEND_GAIN;

  if (!(gain > 0.0F))
  {
    gain = TASK0729_GENERATED_FRONTEND_GAIN;
  }

  return (TASK0729_GENERATED_FRONTEND_GAIN / gain) *
      TASK0729_VOLTAGE_CALIBRATION;
}

static void Task0729_ResetOscillators(uint32_t component_count)
{
  uint32_t component;

  for (component = 0U; component < component_count; ++component)
  {
    task0729_fit.cosine[component] = 1.0;
    task0729_fit.sine[component] = 0.0;
  }
}

static void Task0729_AdvanceOscillators(
    uint32_t component_count, uint32_t sample_index)
{
  uint32_t component;

  for (component = 0U; component < component_count; ++component)
  {
    double cosine = task0729_fit.cosine[component];
    double sine = task0729_fit.sine[component];
    double next_cosine =
        cosine * task0729_fit.cosine_step[component] -
        sine * task0729_fit.sine_step[component];
    double next_sine =
        sine * task0729_fit.cosine_step[component] +
        cosine * task0729_fit.sine_step[component];

    task0729_fit.cosine[component] = next_cosine;
    task0729_fit.sine[component] = next_sine;

    if ((sample_index & 255U) == 0U)
    {
      double magnitude =
          sqrt(next_cosine * next_cosine + next_sine * next_sine);
      if (magnitude > 0.0)
      {
        task0729_fit.cosine[component] /= magnitude;
        task0729_fit.sine[component] /= magnitude;
      }
    }
  }
}

static uint8_t Task0729_SolveNormalEquations(uint32_t dimension)
{
  uint32_t column;

  for (column = 0U; column < dimension; ++column)
  {
    uint32_t pivot = column;
    double pivot_magnitude =
        fabs(task0729_fit.normal[column][column]);
    uint32_t row;

    for (row = column + 1U; row < dimension; ++row)
    {
      double candidate =
          fabs(task0729_fit.normal[row][column]);
      if (candidate > pivot_magnitude)
      {
        pivot = row;
        pivot_magnitude = candidate;
      }
    }

    if (pivot_magnitude <= TASK0729_FIT_PIVOT_EPSILON)
    {
      return 0U;
    }

    if (pivot != column)
    {
      uint32_t entry;
      double temporary;

      for (entry = column; entry < dimension; ++entry)
      {
        temporary = task0729_fit.normal[column][entry];
        task0729_fit.normal[column][entry] =
            task0729_fit.normal[pivot][entry];
        task0729_fit.normal[pivot][entry] = temporary;
      }
      temporary = task0729_fit.rhs[column];
      task0729_fit.rhs[column] = task0729_fit.rhs[pivot];
      task0729_fit.rhs[pivot] = temporary;
    }

    for (row = column + 1U; row < dimension; ++row)
    {
      double factor =
          task0729_fit.normal[row][column] /
          task0729_fit.normal[column][column];
      uint32_t entry;

      task0729_fit.normal[row][column] = 0.0;
      for (entry = column + 1U; entry < dimension; ++entry)
      {
        task0729_fit.normal[row][entry] -=
            factor * task0729_fit.normal[column][entry];
      }
      task0729_fit.rhs[row] -= factor * task0729_fit.rhs[column];
    }
  }

  for (column = dimension; column > 0U; --column)
  {
    uint32_t row = column - 1U;
    double value = task0729_fit.rhs[row];
    uint32_t entry;

    for (entry = row + 1U; entry < dimension; ++entry)
    {
      value -= task0729_fit.normal[row][entry] *
          task0729_fit.solution[entry];
    }
    task0729_fit.solution[row] =
        value / task0729_fit.normal[row][row];
  }

  return 1U;
}

static uint8_t Task0729_RefineAmplitudes(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Result *result)
{
  uint32_t component_count = 0U;
  uint32_t dimension;
  uint32_t result_index;
  uint32_t sample_index;
  double input_scale;
  double minimum = 0.0;
  double maximum = 0.0;
  double square_sum = 0.0;

  memset(&task0729_fit, 0, sizeof(task0729_fit));

  for (result_index = 0U;
       result_index < TASK0729_COMPONENT_COUNT; ++result_index)
  {
    double frequency = (double)result->frequency_hz[result_index];

    if ((frequency > 0.0) &&
        (frequency < ((double)TASK0729_INPUT_SAMPLE_RATE_HZ * 0.5)))
    {
      double angle =
          TASK0729_TWO_PI * frequency /
          (double)TASK0729_INPUT_SAMPLE_RATE_HZ;

      task0729_fit.result_index[component_count] =
          (uint8_t)result_index;
      task0729_fit.cosine_step[component_count] = cos(angle);
      task0729_fit.sine_step[component_count] = sin(angle);
      ++component_count;
    }
  }

  if (component_count == 0U)
  {
    return 0U;
  }

  dimension = 1U + (2U * component_count);
  Task0729_ResetOscillators(component_count);

  for (sample_index = 0U;
       sample_index < TASK0729_INPUT_SAMPLES; ++sample_index)
  {
    uint32_t component;
    uint32_t row;

    task0729_fit.basis[0] = 1.0;
    for (component = 0U; component < component_count; ++component)
    {
      task0729_fit.basis[1U + (2U * component)] =
          task0729_fit.cosine[component];
      task0729_fit.basis[2U + (2U * component)] =
          task0729_fit.sine[component];
    }

    for (row = 0U; row < dimension; ++row)
    {
      uint32_t column;

      task0729_fit.rhs[row] +=
          task0729_fit.basis[row] * (double)samples[sample_index];
      for (column = row; column < dimension; ++column)
      {
        task0729_fit.normal[row][column] +=
            task0729_fit.basis[row] *
            task0729_fit.basis[column];
      }
    }

    Task0729_AdvanceOscillators(
        component_count, sample_index + 1U);
  }

  for (result_index = 1U; result_index < dimension; ++result_index)
  {
    uint32_t column;
    for (column = 0U; column < result_index; ++column)
    {
      task0729_fit.normal[result_index][column] =
          task0729_fit.normal[column][result_index];
    }
  }

  if (Task0729_SolveNormalEquations(dimension) == 0U)
  {
    return 0U;
  }

  input_scale =
      (double)Task0729_SampleToInputVolts(32767) / 32767.0;

  for (result_index = 0U;
       result_index < component_count; ++result_index)
  {
    uint32_t output_index =
        task0729_fit.result_index[result_index];
    double cosine_coefficient =
        task0729_fit.solution[1U + (2U * result_index)];
    double sine_coefficient =
        task0729_fit.solution[2U + (2U * result_index)];

    result->amplitude_vpk[output_index] =
        (float)(sqrt(cosine_coefficient * cosine_coefficient +
                     sine_coefficient * sine_coefficient) *
                input_scale);
  }

  /*
   * Reconstruct the fitted AC waveform to obtain Vpp and RMS without ADC
   * quantization spikes, DC offset, or cross-talk between harmonic fits.
   */
  Task0729_ResetOscillators(component_count);
  for (sample_index = 0U;
       sample_index < TASK0729_INPUT_SAMPLES; ++sample_index)
  {
    double fitted = 0.0;
    uint32_t component;

    for (component = 0U; component < component_count; ++component)
    {
      fitted +=
          task0729_fit.solution[1U + (2U * component)] *
              task0729_fit.cosine[component] +
          task0729_fit.solution[2U + (2U * component)] *
              task0729_fit.sine[component];
    }

    if (sample_index == 0U)
    {
      minimum = fitted;
      maximum = fitted;
    }
    else
    {
      if (fitted < minimum)
      {
        minimum = fitted;
      }
      if (fitted > maximum)
      {
        maximum = fitted;
      }
    }
    square_sum += fitted * fitted;
    Task0729_AdvanceOscillators(
        component_count, sample_index + 1U);
  }

  result->vpp = (float)((maximum - minimum) * input_scale);
  result->vrms = (float)(sqrt(
      square_sum / (double)TASK0729_INPUT_SAMPLES) * input_scale);
  return 1U;
}

const Task0729_Result *Task0729_GetLastResult(void)
{
  return &task0729_last_result;
}

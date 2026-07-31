#include "task0729_processor.h"

#include "G_Export_V4.h"

#include <math.h>
#include <string.h>

#define TASK0729_FIT_MAX_TERMS \
  (1U + (2U * TASK0729_COMPONENT_COUNT))
/* 生成模型内部把ADC电压按4倍前端增益换算回输入。 */
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
static Task0729_Result task0729_current_result;
static Task0729_FitWorkspace task0729_fit;
static uint8_t task0729_history_valid;

static float Task0729_OutputScaleCorrection(void);
static void Task0729_ResetOscillators(uint32_t component_count);
static void Task0729_AdvanceOscillators(
    uint32_t component_count, uint32_t sample_index);
static uint8_t Task0729_SolveNormalEquations(uint32_t dimension);
static uint8_t Task0729_RefineAmplitudes(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Result *result);

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
  memset(&task0729_current_result, 0, sizeof(task0729_current_result));
  task0729_history_valid = 0U;
  G_Export_V4_initialize();
}

uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    uint8_t periods,
    Task0729_Result *result)
{
  uint32_t index;
  uint8_t same_layout;
  float output_scale;
  float setting_ratio[TASK0729_COMPONENT_COUNT];
  Task0729_Result *current = &task0729_current_result;

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

  /* 1. 把一整帧ADC数据交给Simulink生成代码。 */
  memcpy(G_Export_V4_U.adc_block, samples,
         sizeof(G_Export_V4_U.adc_block));
  /* 统一使用题目3：所有工况都走抗干扰和幅值补偿路径。 */
  G_Export_V4_U.mode = (uint8_T)TASK0729_MODE_QUESTION_3;
  G_Export_V4_U.periods = periods;
  /* 2. 执行FFT、谐波提取和题目3的抗干扰处理。 */
  G_Export_V4_step();

  /* 3. 先把模型输出换算成实际输入端电压。 */
  output_scale = Task0729_OutputScaleCorrection();
  memset(current, 0, sizeof(*current));
  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    float generated_amplitude = G_Export_V4_Y.amplitude_Vpk[index];

    current->frequency_hz[index] =
        G_Export_V4_Y.frequency_Hz[index];
    current->amplitude_vpk[index] =
        generated_amplitude * output_scale;
    current->amplitude_setting_vpk[index] =
        G_Export_V4_Y.amplitude_SettingVpk[index] * output_scale;
    current->harmonic_order[index] =
        G_Export_V4_Y.harmonic_order[index];
    setting_ratio[index] = 1.0F;
    if (generated_amplitude > 1.0E-9F)
    {
      setting_ratio[index] =
          G_Export_V4_Y.amplitude_SettingVpk[index] /
          generated_amplitude;
    }
  }
  current->component_count = G_Export_V4_Y.component_count;
  current->vpp = G_Export_V4_Y.Vpp * output_scale;
  current->vrms = G_Export_V4_Y.Vrms * output_scale;
  current->fundamental_hz = G_Export_V4_Y.fundamental_Hz;
  current->waveform_count = G_Export_V4_Y.waveCount;
  for (index = 0U; index < TASK0729_WAVEFORM_SAMPLES; ++index)
  {
    current->waveform[index] =
        G_Export_V4_Y.waveform[index] * output_scale;
  }

  /*
   * 4. 用原始ADC整帧重新拟合幅值，减少FFT栅栏误差。
   * Jointly refit all detected tones against the original 8 MSPS block.
   * This reduces cross-talk between strong adjacent components and removes
   * DC-offset bias.  Keep Vpp, Vrms and waveform from the generated path so
   * they continue to describe the actual filtered ADC input.
   */
  (void)Task0729_RefineAmplitudes(samples, current);
  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    current->amplitude_setting_vpk[index] =
        current->amplitude_vpk[index] * setting_ratio[index];
  }

  same_layout =
      ((task0729_history_valid != 0U) &&
       (task0729_last_result.component_count ==
        current->component_count)) ? 1U : 0U;
  if (same_layout != 0U)
  {
    for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
    {
      if (task0729_last_result.harmonic_order[index] !=
          current->harmonic_order[index])
      {
        same_layout = 0U;
        break;
      }
    }
  }

  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    float frequency = current->frequency_hz[index];
    float amplitude = current->amplitude_vpk[index];
    float setting_amplitude =
        current->amplitude_setting_vpk[index];

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
        current->harmonic_order[index];
  }
  task0729_last_result.component_count =
      current->component_count;
  if (same_layout != 0U)
  {
    task0729_last_result.vpp = Task0729_SmoothStableValue(
        task0729_last_result.vpp, current->vpp);
    task0729_last_result.vrms = Task0729_SmoothStableValue(
        task0729_last_result.vrms, current->vrms);
    task0729_last_result.fundamental_hz =
        Task0729_SmoothStableValue(
            task0729_last_result.fundamental_hz,
            current->fundamental_hz);
  }
  else
  {
    task0729_last_result.vpp = current->vpp;
    task0729_last_result.vrms = current->vrms;
    task0729_last_result.fundamental_hz =
        current->fundamental_hz;
  }
  task0729_last_result.waveform_count =
      current->waveform_count;
  memcpy(task0729_last_result.waveform,
         current->waveform,
         sizeof(task0729_last_result.waveform));

  task0729_history_valid = 1U;
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
  /* 实测前端增益在头文件中改；这里不要改生成模型的4.0。 */
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

  /* 用直流项+每个频率的正弦/余弦项做最小二乘拟合。 */
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
   * Reconstruct only the detected AC components.  This keeps Vpp and RMS in
   * actual input volts while rejecting DC offset, isolated ADC glitches and
   * FIR startup transients.  The normalization recovery factor is not used.
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

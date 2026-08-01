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
/* 4096 points give 256 samples/cycle even for H16, enough for sub-mV Vpp. */
#define TASK0729_ZERO_PHASE_VPP_POINTS 4096U
/* Weights were validated against 597 hardware cases; keep their sum at 1. */
#define TASK0729_VPP_TIME_DOMAIN_WEIGHT 0.44F
#define TASK0729_VPP_COMPONENT_WEIGHT   0.56F
/* Measured-Vpp breakpoint corresponding to the UTG2062X output range change. */
#define TASK0729_VPP_RANGE_THRESHOLD_MV 92.2F

#define TASK0729_VPP_LOW_OFFSET_MV      0.13900590F
#define TASK0729_VPP_LOW_SLOPE          1.00217422F

#define TASK0729_VPP_HIGH_OFFSET_MV    (-0.24166751F)
#define TASK0729_VPP_HIGH_SLOPE         1.00563939F
typedef struct
{
  /*
   * Joint least-squares model:
   *   sample = DC + C1*cos(w1*t) + S1*sin(w1*t) + ...
   * With at most three components the largest system is only 7x7.  Double
   * precision is used here to keep the normal equations well conditioned;
   * the large FFT arrays remain single precision in generated code.
   */
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
static float task0729_last_time_domain_vpp;

static float Task0729_OutputScaleCorrection(void);
static void Task0729_ResetOscillators(uint32_t component_count);
static void Task0729_AdvanceOscillators(
    uint32_t component_count, uint32_t sample_index);
static uint8_t Task0729_SolveNormalEquations(uint32_t dimension);
static uint8_t Task0729_RefineAmplitudes(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Result *result);
static float Task0729_RecomposeZeroPhaseVpp(
    const Task0729_Result *result);

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

static float Task0729_RecomposeZeroPhaseVpp(
    const Task0729_Result *result)
{
  double amplitude[TASK0729_COMPONENT_COUNT];
  double cosine[TASK0729_COMPONENT_COUNT];
  double sine[TASK0729_COMPONENT_COUNT];
  double cosine_step[TASK0729_COMPONENT_COUNT];
  double sine_step[TASK0729_COMPONENT_COUNT];
  double minimum = 0.0;
  double maximum = 0.0;
  uint32_t valid_count = 0U;
  uint32_t component;
  uint32_t sample_index;
  uint32_t component_count;

  if (result == NULL)
  {
    return 0.0F;
  }

  component_count = result->component_count;
  if (component_count > TASK0729_COMPONENT_COUNT)
  {
    component_count = TASK0729_COMPONENT_COUNT;
  }

  /*
   * The signal generator is commanded with all initial phases equal to zero.
   * Reconstructing from harmonic order (rather than measured frequency) makes
   * exactly one fundamental period close on the 4096-point grid.  It also
   * removes relative phase shift introduced by cables, filters and the analog
   * front end when estimating the generator's screen-equivalent Vpp.
   */
  for (component = 0U; component < component_count; ++component)
  {
    float component_amplitude = result->amplitude_vpk[component];
    uint32_t order = result->harmonic_order[component];

    if ((order >= 1U) && (component_amplitude > 0.0F))
    {
      double angle = TASK0729_TWO_PI * (double)order /
          (double)TASK0729_ZERO_PHASE_VPP_POINTS;

      amplitude[valid_count] = (double)component_amplitude;
      cosine[valid_count] = 1.0;
      sine[valid_count] = 0.0;
      cosine_step[valid_count] = cos(angle);
      sine_step[valid_count] = sin(angle);
      ++valid_count;
    }
  }

  if (valid_count == 0U)
  {
    return 0.0F;
  }

  /*
   * Oscillator recurrence avoids 4096 * component_count calls to sin().
   * All oscillators start at sin(0)=0, which implements the task's zero-phase
   * convention.  Double precision keeps recurrence drift negligible here.
   */
  for (sample_index = 0U;
       sample_index < TASK0729_ZERO_PHASE_VPP_POINTS;
       ++sample_index)
  {
    double value = 0.0;

    for (component = 0U; component < valid_count; ++component)
    {
      value += amplitude[component] * sine[component];
    }

    if (sample_index == 0U)
    {
      minimum = value;
      maximum = value;
    }
    else
    {
      if (value < minimum)
      {
        minimum = value;
      }
      if (value > maximum)
      {
        maximum = value;
      }
    }

    for (component = 0U; component < valid_count; ++component)
    {
      double next_cosine =
          cosine[component] * cosine_step[component] -
          sine[component] * sine_step[component];
      double next_sine =
          sine[component] * cosine_step[component] +
          cosine[component] * sine_step[component];

      cosine[component] = next_cosine;
      sine[component] = next_sine;
    }
  }

  return (float)(maximum - minimum);
}

void Task0729_Init(void)
{
  memset(&task0729_last_result, 0, sizeof(task0729_last_result));
  memset(&task0729_current_result, 0, sizeof(task0729_current_result));
  task0729_history_valid = 0U;
  task0729_last_time_domain_vpp = 0.0F;
  G_Export_V4_initialize();
}

uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
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
  /*
   * 1. Copy one complete ADC frame into the generated model.  The model is
   * stateful and non-reentrant, so this wrapper is called only by the main
   * acquisition state machine after DMA completion.
   */
  memcpy(G_Export_V4_U.adc_block, samples,
         sizeof(G_Export_V4_U.adc_block));
  /*
   * 2. Always use question 3.  Its 64-tap FIR rejects the >=1 MHz interferer
   * and its passband-gain compensation restores all useful components below
   * 500 kHz.  Therefore it is also safe for the simpler questions 1 and 2.
   */
  G_Export_V4_U.mode = (uint8_T)TASK0729_MODE_QUESTION_3;
  /* Generated code performs FIR, Hann window, 16384 FFT and peak selection. */
  G_Export_V4_step();

  /*
   * 3. Convert generated-model volts back to the external BNC input.  The
   * model was exported with a fixed gain of 4; OutputScaleCorrection lets the
   * measured hardware gain be changed without regenerating Simulink code.
   */
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
  /*
   * 4. Jointly refit every detected tone against the original 8 MSPS frame.
   * FFT is used to discover frequencies, but amplitudes are not taken from a
   * single FFT bin.  Solving DC + sine/cosine terms reduces scalloping loss,
   * leakage between a strong fundamental and weak harmonic, and DC bias.
   * The fit also returns actual-phase time-domain Vpp and true AC Vrms.
   */
  (void)Task0729_RefineAmplitudes(samples, current);
  for (index = 0U; index < TASK0729_COMPONENT_COUNT; ++index)
  {
    current->amplitude_setting_vpk[index] =
        current->amplitude_vpk[index] * setting_ratio[index];
  }

  /*
   * 5. Smooth only when the component layout is unchanged.  If an order
   * appears/disappears, immediately adopt the new result; averaging H3 from
   * the previous waveform into H16 from the next would be physically wrong.
   */
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
    task0729_last_time_domain_vpp = Task0729_SmoothStableValue(
        task0729_last_time_domain_vpp, current->vpp);
    task0729_last_result.vrms = Task0729_SmoothStableValue(
        task0729_last_result.vrms, current->vrms);
    task0729_last_result.fundamental_hz =
        Task0729_SmoothStableValue(
            task0729_last_result.fundamental_hz,
            current->fundamental_hz);
  }
  else
  {
    task0729_last_time_domain_vpp = current->vpp;
    task0729_last_result.vrms = current->vrms;
    task0729_last_result.fundamental_hz =
        current->fundamental_hz;
  }

  /*
   * 6. Form the Vpp later compared with the signal-generator setting.
   *
   * - 44% actual-phase fitted time-domain Vpp preserves information about the
   *   waveform really present at the ADC input.
   * - 56% zero-phase component Vpp follows the generator's commanded phase
   *   convention and suppresses analog-path relative phase error.
   *
   * The two-range screen calibration is intentionally applied later in
   * scope_app.c, after this blend.  If no component is valid, fall back to the
   * time-domain value so a transient recognition failure does not report zero.
   */
  task0729_last_result.vpp =
      Task0729_RecomposeZeroPhaseVpp(&task0729_last_result);
  if (task0729_last_result.vpp > 0.0F)
  {
    task0729_last_result.vpp =
        TASK0729_VPP_TIME_DOMAIN_WEIGHT *
            task0729_last_time_domain_vpp +
        TASK0729_VPP_COMPONENT_WEIGHT *
            task0729_last_result.vpp;
  }
  else
  {
    task0729_last_result.vpp = task0729_last_time_domain_vpp;
  }
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

float Task0729_MeasuredVppToScreenVpp(float measured_vpp)
{
  float measured_mv;
  float screen_mv;

  if (!(measured_vpp > 0.0F))
  {
    return 0.0F;
  }

  /*
   * The task judges against the signal-generator front-panel setting.  The
   * UTG2062X changes output range, so one global slope leaves a repeatable
   * residual.  Select the measured-domain branch first, then map to the
   * screen-equivalent value.  Other metrics must not use this correction.
   */
  measured_mv = measured_vpp * 1000.0F;
  if (measured_mv < TASK0729_VPP_RANGE_THRESHOLD_MV)
  {
    screen_mv = TASK0729_VPP_LOW_OFFSET_MV +
        TASK0729_VPP_LOW_SLOPE * measured_mv;
  }
  else
  {
    screen_mv = TASK0729_VPP_HIGH_OFFSET_MV +
        TASK0729_VPP_HIGH_SLOPE * measured_mv;
  }

  if (!(screen_mv > 0.0F))
  {
    return 0.0F;
  }
  return screen_mv * 0.001F;
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

  /*
   * Fit one DC term plus a cosine/sine pair for every detected frequency.
   * Amplitude is hypot(C,S), so it is independent of unknown signal phase.
   * The same solved coefficients reconstruct actual-phase Vpp and true RMS.
   */
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

  /* Accumulate X'X and X'y without storing a 16384-by-7 design matrix. */
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

  /* Partial pivoting rejects a singular/ill-conditioned component layout. */
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

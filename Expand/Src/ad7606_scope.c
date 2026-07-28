#include "ad7606_scope.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lcd_spi_154.h"
#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>

/*
 * The acquisition task runs continuously at the AD7606 conversion rate.
 * A 4096-sample ring gives the LCD task enough history to find a clean
 * rising-edge trigger without slowing the converter down.
 */
#define SCOPE_RING_SIZE               4096U
#define SCOPE_RING_MASK               (SCOPE_RING_SIZE - 1U)
#define SCOPE_SNAPSHOT_LIMIT          3072U
#define SCOPE_SNAPSHOT_RETRIES        3U

#define SCOPE_GRAPH_X                 8U
#define SCOPE_GRAPH_Y                 44U
#define SCOPE_GRAPH_WIDTH             224U
#define SCOPE_GRAPH_HEIGHT            192U
#define SCOPE_BLIT_ROWS               16U
#define SCOPE_HORIZONTAL_DIVS         8U
#define SCOPE_VERTICAL_DIVS           8U
#define SCOPE_DEFAULT_MV_PER_DIV      200U
#define SCOPE_DEFAULT_REFRESH_MS      100U

#define SCOPE_COLOR_BLACK             0x0000U
#define SCOPE_COLOR_GRID              0x2125U
#define SCOPE_COLOR_AXIS              0x4208U
#define SCOPE_COLOR_BORDER            0x7BEFU
#define SCOPE_COLOR_WAVE              0xFFE0U

#define SCOPE_D2_RAM                  __attribute__((section(".scope_ram"), aligned(32)))

static int16_t scope_ring_samples[SCOPE_RING_SIZE] SCOPE_D2_RAM;
static uint32_t scope_ring_cycles[SCOPE_RING_SIZE] SCOPE_D2_RAM;
static int16_t scope_snapshot_samples[SCOPE_RING_SIZE] SCOPE_D2_RAM;
static uint32_t scope_snapshot_cycles[SCOPE_RING_SIZE] SCOPE_D2_RAM;
static int16_t scope_auto_samples[SCOPE_SNAPSHOT_LIMIT] SCOPE_D2_RAM;
static uint16_t scope_graph[SCOPE_GRAPH_WIDTH * SCOPE_GRAPH_HEIGHT] SCOPE_D2_RAM;

static volatile uint32_t scope_write_count;
static uint32_t scope_decimation_count;
static uint32_t scope_full_scale_mv = 5000U;
static volatile uint32_t scope_channel_index;
static volatile uint32_t scope_mv_per_div;
static volatile uint32_t scope_decimation;
static volatile int32_t scope_center_mv;
static volatile uint32_t scope_time_per_div_us;
static volatile uint32_t scope_refresh_ms;
static volatile uint32_t scope_input_sample_rate_hz;
static volatile uint8_t scope_running;
static volatile uint8_t scope_center_auto;

static uint32_t ScopeTakeSnapshot(int16_t *samples, uint32_t *cycles);
static void ScopeAnalyzeAndRender(uint32_t count,
                                  const AD7606_ScopeConfig *config);
static uint32_t ScopeIntegerSqrt(uint64_t value);
static int32_t ScopeRawToMv(int32_t raw);
static uint16_t ScopeSampleToY(int16_t raw, uint32_t mv_per_div,
                               int32_t center_mv);
static void ScopeClearGraph(void);
static void ScopeDrawGrid(void);
static void ScopePutPixel(int32_t x, int32_t y, uint16_t color);
static void ScopeDrawLine(int32_t x0, int32_t y0,
                          int32_t x1, int32_t y1, uint16_t color);
static void ScopeDisplayText(uint16_t y, const char *text);
static void ScopeBlitGraph(void);

void AD7606_ScopeInit(uint32_t full_scale_mv)
{
  scope_full_scale_mv = (full_scale_mv == 10000U) ? 10000U : 5000U;
  scope_write_count = 0U;
  scope_decimation_count = 0U;
  scope_channel_index = 0U;
  scope_mv_per_div = SCOPE_DEFAULT_MV_PER_DIV;
  scope_decimation = AD7606_SCOPE_DECIMATION;
  scope_center_mv = 0;
  scope_time_per_div_us = 0U;
  scope_refresh_ms = SCOPE_DEFAULT_REFRESH_MS;
  scope_input_sample_rate_hz = 0U;
  scope_running = 1U;
  scope_center_auto = 0U;

  /* CYCCNT provides sub-microsecond timestamps without consuming a timer. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void AD7606_ScopePushFrame(const int16_t *channels)
{
  uint32_t decimation;
  uint32_t channel;
  uint32_t write_count;
  uint32_t index;

  if ((channels == NULL) || (scope_running == 0U))
  {
    return;
  }

  ++scope_decimation_count;
  decimation = scope_decimation;
  if (scope_decimation_count < decimation)
  {
    return;
  }
  scope_decimation_count = 0U;

  channel = scope_channel_index;
  write_count = scope_write_count;
  index = write_count & SCOPE_RING_MASK;

  scope_ring_samples[index] = channels[channel];
  scope_ring_cycles[index] = DWT->CYCCNT;
  __DMB();
  scope_write_count = write_count + 1U;
}

void AD7606_ScopeGetConfig(AD7606_ScopeConfig *config)
{
  if (config == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  config->channel = (uint8_t)(scope_channel_index + 1U);
  config->running = scope_running;
  config->center_auto = scope_center_auto;
  config->reserved = 0U;
  config->mv_per_div = (uint16_t)scope_mv_per_div;
  config->decimation = (uint16_t)scope_decimation;
  config->center_mv = scope_center_mv;
  config->time_per_div_us = scope_time_per_div_us;
  config->refresh_ms = scope_refresh_ms;
  taskEXIT_CRITICAL();
}

uint8_t AD7606_ScopeSetChannel(uint32_t channel)
{
  if ((channel < 1U) || (channel > 8U))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  scope_channel_index = channel - 1U;
  scope_write_count = 0U;
  scope_decimation_count = 0U;
  taskEXIT_CRITICAL();
  return 1U;
}

uint8_t AD7606_ScopeSetMvPerDiv(uint32_t mv_per_div)
{
  if ((mv_per_div != 50U) && (mv_per_div != 100U) &&
      (mv_per_div != 200U) && (mv_per_div != 500U) &&
      (mv_per_div != 1000U) && (mv_per_div != 2000U))
  {
    return 0U;
  }

  scope_mv_per_div = mv_per_div;
  return 1U;
}

uint8_t AD7606_ScopeSetDecimation(uint32_t decimation)
{
  if ((decimation != 1U) && (decimation != 2U) &&
      (decimation != 4U) && (decimation != 8U) &&
      (decimation != 16U) && (decimation != 32U) &&
      (decimation != 64U))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  scope_decimation = decimation;
  scope_write_count = 0U;
  scope_decimation_count = 0U;
  taskEXIT_CRITICAL();
  return 1U;
}

uint8_t AD7606_ScopeSetCenterMv(int32_t center_mv)
{
  if ((center_mv < -(int32_t)scope_full_scale_mv) ||
      (center_mv > (int32_t)scope_full_scale_mv))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  scope_center_mv = center_mv;
  scope_center_auto = 0U;
  taskEXIT_CRITICAL();
  return 1U;
}

void AD7606_ScopeSetCenterAuto(uint8_t automatic)
{
  taskENTER_CRITICAL();
  scope_center_auto = (automatic != 0U) ? 1U : 0U;
  taskEXIT_CRITICAL();
}

uint8_t AD7606_ScopeSetTimePerDivUs(uint32_t time_per_div_us)
{
  if ((time_per_div_us != 0U) &&
      ((time_per_div_us < 10U) || (time_per_div_us > 1000000U)))
  {
    return 0U;
  }

  scope_time_per_div_us = time_per_div_us;
  return 1U;
}

uint8_t AD7606_ScopeSetRefreshMs(uint32_t refresh_ms)
{
  if ((refresh_ms < 50U) || (refresh_ms > 2000U))
  {
    return 0U;
  }

  scope_refresh_ms = refresh_ms;
  return 1U;
}

void AD7606_ScopeSetRunning(uint8_t running)
{
  taskENTER_CRITICAL();
  if ((running != 0U) && (scope_running == 0U))
  {
    scope_write_count = 0U;
    scope_decimation_count = 0U;
  }
  scope_running = (running != 0U) ? 1U : 0U;
  taskEXIT_CRITICAL();
}

uint32_t AD7606_ScopeGetRefreshMs(void)
{
  return scope_refresh_ms;
}

uint32_t AD7606_ScopeGetInputSampleRateHz(void)
{
  return scope_input_sample_rate_hz;
}

uint8_t AD7606_ScopeAutoConfigure(void)
{
  static const uint32_t vdiv_options[] =
      {50U, 100U, 200U, 500U, 1000U, 2000U};
  uint32_t count = ScopeTakeSnapshot(scope_auto_samples, NULL);
  int16_t minimum;
  int16_t maximum;
  int32_t midpoint;
  int32_t positive_peak;
  int32_t negative_peak;
  uint32_t peak_mv;
  uint32_t selected_vdiv = 2000U;

  if (count < SCOPE_GRAPH_WIDTH)
  {
    return 0U;
  }

  minimum = scope_auto_samples[0];
  maximum = scope_auto_samples[0];
  for (uint32_t i = 0U; i < count; ++i)
  {
    int32_t value = scope_auto_samples[i];
    if (value < minimum)
    {
      minimum = (int16_t)value;
    }
    if (value > maximum)
    {
      maximum = (int16_t)value;
    }
  }
  midpoint = ((int32_t)maximum + minimum) / 2;
  positive_peak = (int32_t)maximum - midpoint;
  negative_peak = midpoint - (int32_t)minimum;
  peak_mv = (uint32_t)ScopeRawToMv(
      (positive_peak > negative_peak) ? positive_peak : negative_peak);

  for (uint32_t i = 0U;
       i < (sizeof(vdiv_options) / sizeof(vdiv_options[0])); ++i)
  {
    if (peak_mv <= (vdiv_options[i] * 3U))
    {
      selected_vdiv = vdiv_options[i];
      break;
    }
  }

  /*
   * DEC controls work performed by the 200 kSPS BUSY interrupt. Keep the
   * user's current value here: changing it from AUTO can abruptly multiply
   * ISR load and starve the scheduler before the command can reply or save.
   * Automatic timebase selection is handled by the LCD renderer.
   */
  taskENTER_CRITICAL();
  scope_mv_per_div = selected_vdiv;
  scope_center_mv = ScopeRawToMv(midpoint);
  scope_center_auto = 1U;
  scope_time_per_div_us = 0U;
  taskEXIT_CRITICAL();
  return 1U;
}

void AD7606_ScopeDisplayInit(void)
{
  LCD_SetDirection(Direction_V);
  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_SetAsciiFont(&ASCII_Font12);
  LCD_ShowNumMode(Fill_Space);
  LCD_Clear();

  ScopeDisplayText(2U, "CH1  200mV/div  Fs:---");
  ScopeDisplayText(16U, "Max:---mV Min:---mV");
  ScopeDisplayText(30U, "Mid:---mV Vpp:---mV RMS:---mV");
  ScopeDisplayText(238U, "F:---Hz DC:---mV C:+0mV");

  ScopeClearGraph();
  ScopeDrawGrid();
  ScopeBlitGraph();
}

void AD7606_ScopeDisplayRefresh(void)
{
  AD7606_ScopeConfig config;
  uint32_t count =
      ScopeTakeSnapshot(scope_snapshot_samples, scope_snapshot_cycles);

  AD7606_ScopeGetConfig(&config);
  if (count < SCOPE_GRAPH_WIDTH)
  {
    char line[40];
    (void)snprintf(line, sizeof(line), "%sCH%u  waiting for signal",
                   (config.running != 0U) ? "" : "HOLD ",
                   config.channel);
    ScopeDisplayText(2U, line);
    return;
  }

  ScopeAnalyzeAndRender(count, &config);
}

static uint32_t ScopeTakeSnapshot(int16_t *samples, uint32_t *cycles)
{
  /*
   * Keep enough unused ring entries ahead of the snapshot so the 200 kSPS
   * BUSY interrupt can continue writing while the LCD copies history. This
   * avoids a long critical section that would otherwise hide BUSY edges.
   */
  for (uint32_t attempt = 0U; attempt < SCOPE_SNAPSHOT_RETRIES; ++attempt)
  {
    uint32_t end = scope_write_count;
    uint32_t count = (end < SCOPE_SNAPSHOT_LIMIT) ?
                     end : SCOPE_SNAPSHOT_LIMIT;
    uint32_t first = end - count;

    for (uint32_t i = 0U; i < count; ++i)
    {
      uint32_t source = (first + i) & SCOPE_RING_MASK;
      samples[i] = scope_ring_samples[source];
      if (cycles != NULL)
      {
        cycles[i] = scope_ring_cycles[source];
      }
    }

    __DMB();
    if ((scope_write_count - end) <= (SCOPE_RING_SIZE - count))
    {
      return count;
    }
  }
  return 0U;
}

static void ScopeAnalyzeAndRender(uint32_t count,
                                  const AD7606_ScopeConfig *config)
{
  int16_t minimum = scope_snapshot_samples[0];
  int16_t maximum = scope_snapshot_samples[0];
  int64_t sum = 0;
  uint64_t sum_squares = 0U;
  int32_t mean;
  int32_t midpoint;
  int32_t display_center_mv;
  uint64_t variance;
  uint32_t rms_raw;
  uint32_t display_sample_rate_hz = 0U;
  uint32_t input_sample_rate_hz = 0U;
  uint32_t frequency_millihz = 0U;
  uint32_t crossings[16];
  uint32_t crossing_count = 0U;
  uint32_t period_samples = 0U;
  uint32_t plot_span = SCOPE_GRAPH_WIDTH - 1U;
  uint32_t plot_start = count - SCOPE_GRAPH_WIDTH;
  int32_t hysteresis;
  uint8_t trigger_armed = 0U;
  char line[64];
  char timebase[16];

  for (uint32_t i = 0U; i < count; ++i)
  {
    int32_t value = scope_snapshot_samples[i];
    if (value < minimum)
    {
      minimum = (int16_t)value;
    }
    if (value > maximum)
    {
      maximum = (int16_t)value;
    }
    sum += value;
    sum_squares += (uint64_t)((int64_t)value * value);
  }

  mean = (int32_t)(sum / (int64_t)count);
  midpoint = ((int32_t)maximum + (int32_t)minimum) / 2;
  display_center_mv = (config->center_auto != 0U) ?
                      ScopeRawToMv(mean) : config->center_mv;
  {
    int64_t mean_square = (int64_t)(sum_squares / count);
    int64_t dc_square = (int64_t)mean * mean;
    variance = (mean_square > dc_square) ?
               (uint64_t)(mean_square - dc_square) : 0U;
  }
  rms_raw = ScopeIntegerSqrt(variance);

  {
    uint32_t elapsed_cycles =
        scope_snapshot_cycles[count - 1U] - scope_snapshot_cycles[0];
    if (elapsed_cycles != 0U)
    {
      display_sample_rate_hz = (uint32_t)
          (((uint64_t)(count - 1U) * SystemCoreClock) / elapsed_cycles);
    }
  }
  input_sample_rate_hz = display_sample_rate_hz * config->decimation;
  scope_input_sample_rate_hz = input_sample_rate_hz;

  /*
   * A Schmitt-style trigger rejects small zero-crossing noise. The trigger
   * follows the measured DC mean independently of the selected display
   * center voltage.
   */
  hysteresis = ((int32_t)maximum - (int32_t)minimum) / 20;
  if (hysteresis < 8)
  {
    hysteresis = 8;
  }

  for (uint32_t i = 0U; i < count; ++i)
  {
    int32_t value = scope_snapshot_samples[i];
    if (value <= (mean - hysteresis))
    {
      trigger_armed = 1U;
    }
    else if ((trigger_armed != 0U) && (value >= (mean + hysteresis)))
    {
      if (crossing_count < (sizeof(crossings) / sizeof(crossings[0])))
      {
        crossings[crossing_count++] = i;
      }
      else
      {
        memmove(&crossings[0], &crossings[1],
                (sizeof(crossings) / sizeof(crossings[0]) - 1U) *
                sizeof(crossings[0]));
        crossings[(sizeof(crossings) / sizeof(crossings[0])) - 1U] = i;
      }
      trigger_armed = 0U;
    }
  }

  if (crossing_count >= 2U)
  {
    uint32_t first_period = (crossing_count > 6U) ?
                            (crossing_count - 6U) : 1U;
    uint32_t periods = crossing_count - first_period;
    uint32_t total_samples =
        crossings[crossing_count - 1U] - crossings[first_period - 1U];
    uint32_t total_cycles =
        scope_snapshot_cycles[crossings[crossing_count - 1U]] -
        scope_snapshot_cycles[crossings[first_period - 1U]];

    if ((periods != 0U) && (total_samples != 0U))
    {
      period_samples = total_samples / periods;
    }
    if ((periods != 0U) && (total_cycles != 0U))
    {
      uint32_t average_period_cycles = total_cycles / periods;
      if (average_period_cycles != 0U)
      {
        frequency_millihz = (uint32_t)
            (((uint64_t)SystemCoreClock * 1000U) / average_period_cycles);
      }
    }
  }

  {
    uint64_t desired_span = 0U;

    if ((config->time_per_div_us != 0U) &&
        (display_sample_rate_hz != 0U))
    {
      desired_span = ((uint64_t)display_sample_rate_hz *
                      config->time_per_div_us *
                      SCOPE_HORIZONTAL_DIVS) / 1000000U;
      if (desired_span == 0U)
      {
        desired_span = 1U;
      }
    }
    else if (period_samples != 0U)
    {
      desired_span = (uint64_t)period_samples * 2U;
    }

    if (desired_span != 0U)
    {
      if (desired_span > (count - 1U))
      {
        desired_span = count - 1U;
      }
      if (desired_span == 0U)
      {
        desired_span = 1U;
      }
      plot_span = (uint32_t)desired_span;
    }
  }

  {
    uint32_t required_pre = plot_span / 8U;
    uint32_t required_post = plot_span - required_pre;

    plot_start = count - 1U - plot_span;

    for (uint32_t n = crossing_count; n > 0U; --n)
    {
      uint32_t crossing = crossings[n - 1U];
      if ((crossing >= required_pre) &&
          ((count - 1U - crossing) >= required_post))
      {
        plot_start = crossing - required_pre;
        break;
      }
    }
  }

  ScopeClearGraph();
  ScopeDrawGrid();

  {
    uint16_t previous_y =
        ScopeSampleToY(scope_snapshot_samples[plot_start],
                       config->mv_per_div, display_center_mv);
    for (uint32_t x = 1U; x < SCOPE_GRAPH_WIDTH; ++x)
    {
      uint32_t sample_index = plot_start + (uint32_t)
          (((uint64_t)x * plot_span) / (SCOPE_GRAPH_WIDTH - 1U));
      uint16_t y = ScopeSampleToY(scope_snapshot_samples[sample_index],
                                  config->mv_per_div, display_center_mv);
      ScopeDrawLine((int32_t)x - 1, previous_y,
                    (int32_t)x, y, SCOPE_COLOR_WAVE);
      previous_y = y;
    }
  }

  ScopeBlitGraph();

  if (config->time_per_div_us == 0U)
  {
    (void)snprintf(timebase, sizeof(timebase), "AUTO");
  }
  else if ((config->time_per_div_us >= 1000U) &&
           ((config->time_per_div_us % 1000U) == 0U))
  {
    (void)snprintf(timebase, sizeof(timebase), "%lums/d",
                   (unsigned long)(config->time_per_div_us / 1000U));
  }
  else
  {
    (void)snprintf(timebase, sizeof(timebase), "%luus/d",
                   (unsigned long)config->time_per_div_us);
  }

  if (input_sample_rate_hz >= 1000U)
  {
    (void)snprintf(line, sizeof(line), "%sCH%u %umV/d %s Fs:%lu.%luk",
                   (config->running != 0U) ? "" : "HOLD ",
                   (unsigned int)config->channel,
                   (unsigned int)config->mv_per_div, timebase,
                   (unsigned long)(input_sample_rate_hz / 1000U),
                   (unsigned long)((input_sample_rate_hz % 1000U) / 100U));
  }
  else
  {
    (void)snprintf(line, sizeof(line), "%sCH%u %umV/d %s Fs:%luHz",
                   (config->running != 0U) ? "" : "HOLD ",
                   (unsigned int)config->channel,
                   (unsigned int)config->mv_per_div, timebase,
                   (unsigned long)input_sample_rate_hz);
  }
  ScopeDisplayText(2U, line);

  (void)snprintf(line, sizeof(line), "Max:%+ldmV Min:%+ldmV",
                 (long)ScopeRawToMv(maximum),
                 (long)ScopeRawToMv(minimum));
  ScopeDisplayText(16U, line);

  (void)snprintf(line, sizeof(line),
                 "Mid:%+ldmV Vpp:%ldmV RMS:%ldmV",
                 (long)ScopeRawToMv(midpoint),
                 (long)ScopeRawToMv((int32_t)maximum - minimum),
                 (long)ScopeRawToMv((int32_t)rms_raw));
  ScopeDisplayText(30U, line);

  if (frequency_millihz >= 1000000U)
  {
    (void)snprintf(line, sizeof(line), "F:%lu.%lukHz DC:%+ldmV C:%s%+ldmV",
                   (unsigned long)(frequency_millihz / 1000000U),
                   (unsigned long)((frequency_millihz % 1000000U) / 100000U),
                   (long)ScopeRawToMv(mean),
                   (config->center_auto != 0U) ? "A" : "",
                   (long)display_center_mv);
  }
  else if (frequency_millihz != 0U)
  {
    (void)snprintf(line, sizeof(line), "F:%lu.%luHz DC:%+ldmV C:%s%+ldmV",
                   (unsigned long)(frequency_millihz / 1000U),
                   (unsigned long)((frequency_millihz % 1000U) / 100U),
                   (long)ScopeRawToMv(mean),
                   (config->center_auto != 0U) ? "A" : "",
                   (long)display_center_mv);
  }
  else
  {
    (void)snprintf(line, sizeof(line), "F:---Hz DC:%+ldmV C:%s%+ldmV",
                   (long)ScopeRawToMv(mean),
                   (config->center_auto != 0U) ? "A" : "",
                   (long)display_center_mv);
  }
  ScopeDisplayText(238U, line);
}

static uint32_t ScopeIntegerSqrt(uint64_t value)
{
  uint64_t result = 0U;
  uint64_t bit = (uint64_t)1U << 62;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0U)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }

  return (uint32_t)result;
}

static int32_t ScopeRawToMv(int32_t raw)
{
  int64_t scaled = (int64_t)raw * scope_full_scale_mv;
  if (scaled >= 0)
  {
    scaled += 16384;
  }
  else
  {
    scaled -= 16384;
  }
  return (int32_t)(scaled / 32768);
}

static uint16_t ScopeSampleToY(int16_t raw, uint32_t mv_per_div,
                               int32_t center_mv)
{
  int32_t millivolts = ScopeRawToMv(raw) - center_mv;
  int32_t center = (int32_t)(SCOPE_GRAPH_HEIGHT / 2U);
  int32_t pixels_per_div =
      (int32_t)(SCOPE_GRAPH_HEIGHT / SCOPE_VERTICAL_DIVS);
  int32_t y = center -
      (millivolts * pixels_per_div) / (int32_t)mv_per_div;

  if (y < 1)
  {
    y = 1;
  }
  else if (y > ((int32_t)SCOPE_GRAPH_HEIGHT - 2))
  {
    y = (int32_t)SCOPE_GRAPH_HEIGHT - 2;
  }
  return (uint16_t)y;
}

static void ScopeClearGraph(void)
{
  for (uint32_t i = 0U;
       i < (SCOPE_GRAPH_WIDTH * SCOPE_GRAPH_HEIGHT); ++i)
  {
    scope_graph[i] = SCOPE_COLOR_BLACK;
  }
}

static void ScopeDrawGrid(void)
{
  uint32_t x_div = SCOPE_GRAPH_WIDTH / SCOPE_HORIZONTAL_DIVS;
  uint32_t y_div = SCOPE_GRAPH_HEIGHT / SCOPE_VERTICAL_DIVS;

  for (uint32_t division = 0U;
       division <= SCOPE_HORIZONTAL_DIVS; ++division)
  {
    uint32_t x = division * x_div;
    if (x >= SCOPE_GRAPH_WIDTH)
    {
      x = SCOPE_GRAPH_WIDTH - 1U;
    }
    for (uint32_t y = 0U; y < SCOPE_GRAPH_HEIGHT; ++y)
    {
      scope_graph[y * SCOPE_GRAPH_WIDTH + x] =
          ((division == 0U) || (division == SCOPE_HORIZONTAL_DIVS)) ?
          SCOPE_COLOR_BORDER : SCOPE_COLOR_GRID;
    }
  }

  for (uint32_t division = 0U;
       division <= SCOPE_VERTICAL_DIVS; ++division)
  {
    uint32_t y = division * y_div;
    uint16_t color;
    if (y >= SCOPE_GRAPH_HEIGHT)
    {
      y = SCOPE_GRAPH_HEIGHT - 1U;
    }

    if ((division == 0U) || (division == SCOPE_VERTICAL_DIVS))
    {
      color = SCOPE_COLOR_BORDER;
    }
    else if (division == (SCOPE_VERTICAL_DIVS / 2U))
    {
      color = SCOPE_COLOR_AXIS;
    }
    else
    {
      color = SCOPE_COLOR_GRID;
    }

    for (uint32_t x = 0U; x < SCOPE_GRAPH_WIDTH; ++x)
    {
      scope_graph[y * SCOPE_GRAPH_WIDTH + x] = color;
    }
  }
}

static void ScopePutPixel(int32_t x, int32_t y, uint16_t color)
{
  if ((x >= 0) && (x < (int32_t)SCOPE_GRAPH_WIDTH) &&
      (y >= 0) && (y < (int32_t)SCOPE_GRAPH_HEIGHT))
  {
    scope_graph[(uint32_t)y * SCOPE_GRAPH_WIDTH + (uint32_t)x] = color;
  }
}

static void ScopeDrawLine(int32_t x0, int32_t y0,
                          int32_t x1, int32_t y1, uint16_t color)
{
  int32_t dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
  int32_t sx = (x0 < x1) ? 1 : -1;
  int32_t dy_abs = (y1 >= y0) ? (y1 - y0) : (y0 - y1);
  int32_t dy = -dy_abs;
  int32_t sy = (y0 < y1) ? 1 : -1;
  int32_t error = dx + dy;

  for (;;)
  {
    ScopePutPixel(x0, y0, color);
    if ((x0 == x1) && (y0 == y1))
    {
      break;
    }

    {
      int32_t twice_error = 2 * error;
      if (twice_error >= dy)
      {
        error += dy;
        x0 += sx;
      }
      if (twice_error <= dx)
      {
        error += dx;
        y0 += sy;
      }
    }
  }
}

static void ScopeDisplayText(uint16_t y, const char *text)
{
  char padded[40];
  size_t length = strlen(text);

  if (length > 39U)
  {
    length = 39U;
  }
  memset(padded, ' ', sizeof(padded));
  memcpy(padded, text, length);
  padded[39] = '\0';

  LCD_SetBackColor(LCD_BLACK);
  LCD_SetColor(LCD_WHITE);
  LCD_DisplayString(2U, y, padded);
}

static void ScopeBlitGraph(void)
{
  /*
   * The LCD driver's polling transfer has a cumulative one-second timeout.
   * Small strips keep each transaction bounded when the ADC task preempts the
   * LCD task, and avoid leaving only the first part of a frame on the panel.
   */
  for (uint32_t row = 0U; row < SCOPE_GRAPH_HEIGHT;
       row += SCOPE_BLIT_ROWS)
  {
    uint32_t rows = SCOPE_GRAPH_HEIGHT - row;
    if (rows > SCOPE_BLIT_ROWS)
    {
      rows = SCOPE_BLIT_ROWS;
    }

    LCD_CopyBuffer(SCOPE_GRAPH_X, SCOPE_GRAPH_Y + (uint16_t)row,
                   SCOPE_GRAPH_WIDTH, (uint16_t)rows,
                   &scope_graph[row * SCOPE_GRAPH_WIDTH]);
  }
}

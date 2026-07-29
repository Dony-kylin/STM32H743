#include "ad9220.h"

#include <stddef.h>

#define AD9220_CAPTURE_TRANSFER_COUNT \
  (AD9220_MAX_CAPTURE_SAMPLES + AD9220_PIPELINE_DELAY)
#define AD9220_PORT_D_SHIFT               16U
#define AD9220_PORT_D_MASK(pin) \
  ((uint32_t)(pin) << AD9220_PORT_D_SHIFT)
#define AD9220_OTR_MASK                   AD9220_PORT_D_MASK(GPIO_PIN_14)
#define AD9220_PORT_E_DATA_MASK           0x0000FF80UL
#define AD9220_PORT_D_DATA_MASK \
  AD9220_PORT_D_MASK(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_14)
#define AD9220_STABILITY_RETRIES           3U
#define AD9220_CAPTURE_DONE_BUS          0x03U
#define AD9220_EDGE_SPIN_LIMIT           8192U
#define AD9220_CAPTURE_ERROR_TIMEOUT     0x00000001U
#define AD9220_CAPTURE_ERROR_TIMING      0x00000002U
#define AD9220_GLITCH_JUMP_THRESHOLD     8192L
#define AD9220_GLITCH_NEIGHBOR_THRESHOLD 8192L
#define AD9220_BUFFER_RAM \
  __attribute__((section(".ad9220_capture_ram"), aligned(32)))

/*
 * Each word stores GPIOE in bits 0..15 and GPIOD in bits 16..31. The first
 * point is phase-aligned to CLK; later points use a precise 2 MHz DWT schedule
 * and double-read consistency checking prevents split-port tearing.
 */
static volatile uint32_t
    ad9220_gpio_samples[AD9220_CAPTURE_TRANSFER_COUNT]
    AD9220_BUFFER_RAM;

static volatile uint32_t ad9220_requested_samples;
static volatile uint32_t ad9220_captured_samples;
static volatile uint32_t ad9220_sample_rate_hz;
static volatile uint32_t ad9220_capture_error_count;
static volatile uint32_t ad9220_overrange_count;
static volatile uint32_t ad9220_port_e_error_code;
static volatile uint32_t ad9220_port_d_error_code;
static volatile uint32_t ad9220_last_timer_delta;
static volatile uint32_t ad9220_timer_delta_limit;
static volatile uint32_t ad9220_glitch_correction_count;
static volatile uint32_t ad9220_last_progress;
static volatile uint32_t ad9220_last_error_stage;
static volatile uint32_t ad9220_last_clock_level;
static volatile uint8_t ad9220_done_mask;
static volatile uint8_t ad9220_capture_busy;
static volatile uint8_t ad9220_capture_complete;
static uint8_t ad9220_initialized;

static HAL_StatusTypeDef AD9220_GPIO_Init(void);
static void AD9220_CycleCounter_Init(void);
static uint16_t AD9220_PackSample(uint32_t gpio_snapshot);

void AD9220_Init(void)
{
  if (ad9220_initialized != 0U)
  {
    return;
  }

  if (AD9220_GPIO_Init() != HAL_OK)
  {
    ++ad9220_capture_error_count;
    return;
  }

  AD9220_CycleCounter_Init();
  ad9220_sample_rate_hz = AD9220_SAMPLE_RATE_HZ;
  ad9220_initialized = 1U;
}

void AD9220_DeInit(void)
{
  AD9220_AbortCapture();
  ad9220_initialized = 0U;
}

__attribute__((optimize("O3")))
AD9220_Status AD9220_StartCapture(uint32_t sample_count)
{
  uint32_t transfer_count;
  uint32_t expected_core_cycles;
  uint32_t saved_primask;
  uint32_t start_cycle;
  uint32_t next_cycle;
  uint32_t elapsed_cycles;
  uint32_t average_cycles;
  uint32_t spins;
  uint32_t i;
  uint8_t capture_error = 0U;

  if ((sample_count == 0U) ||
      (sample_count > AD9220_MAX_CAPTURE_SAMPLES))
  {
    return AD9220_STATUS_INVALID_ARGUMENT;
  }
  if (ad9220_initialized == 0U)
  {
    return AD9220_STATUS_ERROR;
  }
  if (ad9220_capture_busy != 0U)
  {
    return AD9220_STATUS_BUSY;
  }

  transfer_count = sample_count + AD9220_PIPELINE_DELAY;
  ad9220_requested_samples = sample_count;
  ad9220_captured_samples = 0U;
  ad9220_overrange_count = 0U;
  ad9220_port_e_error_code = 0U;
  ad9220_port_d_error_code = 0U;
  ad9220_done_mask = 0U;
  ad9220_capture_complete = 0U;
  ad9220_capture_busy = 1U;
  ad9220_last_progress = 0U;
  ad9220_last_error_stage = 0U;
  ad9220_last_clock_level =
      ((GPIOD->IDR & AD9220_CLK_Pin) != 0U) ? 1U : 0U;

  expected_core_cycles = SystemCoreClock / AD9220_SAMPLE_RATE_HZ;
  ad9220_last_timer_delta = 0U;
  ad9220_timer_delta_limit =
      expected_core_cycles + (expected_core_cycles / 2U) + 2U;

  /*
   * A complete 16384-point block takes about 8.2 ms. Interrupts are masked
   * only for this acquisition burst so an ISR cannot make the loop skip an
   * ADC edge. The bounded edge waits prevent a stopped TCXO from hanging.
   */
  saved_primask = __get_PRIMASK();
  __disable_irq();
  __DSB();
  __ISB();

  /* Start from a known low clock phase before waiting for the first rise. */
  spins = AD9220_EDGE_SPIN_LIMIT;
  while ((GPIOD->IDR & AD9220_CLK_Pin) != 0U)
  {
    if (--spins == 0U)
    {
      capture_error = 1U;
      ad9220_last_error_stage = 1U;
      break;
    }
  }

  /* Align only the first point to a stable AD9220 falling edge. */
  if (capture_error == 0U)
  {
    spins = AD9220_EDGE_SPIN_LIMIT;
    while ((GPIOD->IDR & AD9220_CLK_Pin) == 0U)
    {
      if (--spins == 0U)
      {
        capture_error = 1U;
        ad9220_last_error_stage = 2U;
        break;
      }
    }
  }
  if (capture_error == 0U)
  {
    spins = AD9220_EDGE_SPIN_LIMIT;
    while ((GPIOD->IDR & AD9220_CLK_Pin) != 0U)
    {
      if (--spins == 0U)
      {
        capture_error = 1U;
        ad9220_last_error_stage = 3U;
        break;
      }
    }
  }

  start_cycle = DWT->CYCCNT;
  next_cycle = start_cycle;
  for (i = 0U; (i < transfer_count) && (capture_error == 0U); ++i)
  {
    uint32_t port_e;
    uint32_t port_d;
    uint32_t confirm_e;
    uint32_t confirm_d;

    /*
     * DWT runs at the 480 MHz CPU clock. An absolute 240-cycle schedule gives
     * exactly 2 MSPS and cannot accumulate loop-execution jitter.
     */
    if (i != 0U)
    {
      next_cycle += expected_core_cycles;
      while ((int32_t)(DWT->CYCCNT - next_cycle) < 0)
      {
      }
    }

    /*
     * Read both GPIO ports twice. If the ADC changes data during the split
     * read, retry inside the 500 ns sample budget and keep one stable pair.
     */
    port_e = GPIOE->IDR;
    port_d = GPIOD->IDR;
    for (uint32_t retry = 0U;
         retry < AD9220_STABILITY_RETRIES;
         ++retry)
    {
      confirm_e = GPIOE->IDR;
      confirm_d = GPIOD->IDR;
      if ((((port_e ^ confirm_e) & AD9220_PORT_E_DATA_MASK) == 0U) &&
          ((((port_d ^ confirm_d) << AD9220_PORT_D_SHIFT) &
            AD9220_PORT_D_DATA_MASK) == 0U))
      {
        break;
      }
      port_e = confirm_e;
      port_d = confirm_d;
    }

    ad9220_gpio_samples[i] =
        (port_d << AD9220_PORT_D_SHIFT) | (port_e & 0xFFFFU);
  }
  elapsed_cycles = DWT->CYCCNT - start_cycle;
  ad9220_last_progress = i;
  ad9220_last_clock_level =
      ((GPIOD->IDR & AD9220_CLK_Pin) != 0U) ? 1U : 0U;

  __DMB();
  if (saved_primask == 0U)
  {
    __enable_irq();
  }

  ad9220_capture_busy = 0U;
  if (capture_error != 0U)
  {
    ++ad9220_capture_error_count;
    ad9220_port_e_error_code = AD9220_CAPTURE_ERROR_TIMEOUT;
    ad9220_port_d_error_code = AD9220_CAPTURE_ERROR_TIMEOUT;
    AD9220_CaptureCompleteCallback();
    return AD9220_STATUS_ERROR;
  }

  average_cycles =
      (elapsed_cycles + (transfer_count / 2U)) / transfer_count;
  ad9220_last_timer_delta = average_cycles;
  if ((ad9220_timer_delta_limit != 0U) &&
      (average_cycles > ad9220_timer_delta_limit))
  {
    ++ad9220_capture_error_count;
    ad9220_port_e_error_code = AD9220_CAPTURE_ERROR_TIMING;
    ad9220_port_d_error_code = AD9220_CAPTURE_ERROR_TIMING;
    ad9220_last_error_stage = 4U;
    AD9220_CaptureCompleteCallback();
    return AD9220_STATUS_ERROR;
  }

  ad9220_done_mask = AD9220_CAPTURE_DONE_BUS;
  ad9220_captured_samples = ad9220_requested_samples;
  ad9220_capture_complete = 1U;
  AD9220_CaptureCompleteCallback();
  return AD9220_STATUS_OK;
}

void AD9220_AbortCapture(void)
{
  ad9220_capture_busy = 0U;
  ad9220_capture_complete = 0U;
  ad9220_done_mask = 0U;
}

uint8_t AD9220_IsCaptureBusy(void)
{
  return ad9220_capture_busy;
}

uint8_t AD9220_IsCaptureComplete(void)
{
  return ad9220_capture_complete;
}

uint32_t AD9220_GetSampleRateHz(void)
{
  return ad9220_sample_rate_hz;
}

uint32_t AD9220_GetCapturedCount(void)
{
  return ad9220_captured_samples;
}

uint32_t AD9220_GetDmaErrorCount(void)
{
  return ad9220_capture_error_count;
}

uint32_t AD9220_GetOverrangeCount(void)
{
  return ad9220_overrange_count;
}

uint32_t AD9220_GetDmaDoneMask(void)
{
  return ad9220_done_mask;
}

uint32_t AD9220_GetPortEDmaErrorCode(void)
{
  return ad9220_port_e_error_code;
}

uint32_t AD9220_GetPortDDmaErrorCode(void)
{
  return ad9220_port_d_error_code;
}

uint32_t AD9220_GetLastTimerDelta(void)
{
  return ad9220_last_timer_delta;
}

uint32_t AD9220_GetTimerDeltaLimit(void)
{
  return ad9220_timer_delta_limit;
}

uint32_t AD9220_GetGlitchCorrectionCount(void)
{
  return ad9220_glitch_correction_count;
}

uint32_t AD9220_GetLastProgress(void)
{
  return ad9220_last_progress;
}

uint32_t AD9220_GetLastErrorStage(void)
{
  return ad9220_last_error_stage;
}

uint32_t AD9220_GetLastClockLevel(void)
{
  return ad9220_last_clock_level;
}

uint16_t AD9220_GetRawSample(uint32_t index)
{
  uint32_t source_index;

  if ((ad9220_capture_complete == 0U) ||
      (index >= ad9220_captured_samples))
  {
    return 0U;
  }

  source_index = index + AD9220_PIPELINE_DELAY;
  return AD9220_PackSample(ad9220_gpio_samples[source_index]);
}

int16_t AD9220_GetSignedSample(uint32_t index)
{
  int32_t centered = (int32_t)AD9220_GetRawSample(index) - 2048;
  return (int16_t)(centered << 4);
}

uint8_t AD9220_GetOverrange(uint32_t index)
{
  uint32_t source_index;

  if ((ad9220_capture_complete == 0U) ||
      (index >= ad9220_captured_samples))
  {
    return 0U;
  }

  source_index = index + AD9220_PIPELINE_DELAY;
  return ((ad9220_gpio_samples[source_index] &
           AD9220_OTR_MASK) != 0U) ? 1U : 0U;
}

uint32_t AD9220_CopySignedSamples(int16_t *destination,
                                  uint32_t capacity)
{
  uint32_t count;
  uint32_t overrange_count = 0U;
  uint32_t glitch_count = 0U;

  if ((destination == NULL) || (ad9220_capture_complete == 0U))
  {
    return 0U;
  }

  count = ad9220_captured_samples;
  if (count > capacity)
  {
    count = capacity;
  }

  for (uint32_t i = 0U; i < count; ++i)
  {
    uint32_t source_index = i + AD9220_PIPELINE_DELAY;
    uint32_t gpio_snapshot = ad9220_gpio_samples[source_index];
    uint16_t raw = AD9220_PackSample(gpio_snapshot);

    destination[i] = (int16_t)(((int32_t)raw - 2048) << 4);
    if ((gpio_snapshot & AD9220_OTR_MASK) != 0U)
    {
      ++overrange_count;
    }
  }

  /*
   * Keep a narrowly targeted isolated-glitch repair for electrical glitches.
   * FMC removes split-port tearing, so GLITCH_FIX should normally stay near 0.
   */
  for (uint32_t i = 1U; (i + 1U) < count; ++i)
  {
    int32_t previous = destination[i - 1U];
    int32_t current = destination[i];
    int32_t next = destination[i + 1U];
    int32_t jump_before = current - previous;
    int32_t jump_after = current - next;
    int32_t neighbor_delta = previous - next;
    uint32_t source_index = i + AD9220_PIPELINE_DELAY;

    if (jump_before < 0)
    {
      jump_before = -jump_before;
    }
    if (jump_after < 0)
    {
      jump_after = -jump_after;
    }
    if (neighbor_delta < 0)
    {
      neighbor_delta = -neighbor_delta;
    }

    if ((jump_before > AD9220_GLITCH_JUMP_THRESHOLD) &&
        (jump_after > AD9220_GLITCH_JUMP_THRESHOLD) &&
        (neighbor_delta < AD9220_GLITCH_NEIGHBOR_THRESHOLD) &&
        ((ad9220_gpio_samples[source_index] &
          AD9220_OTR_MASK) == 0U))
    {
      destination[i] = (int16_t)((previous + next) / 2L);
      ++glitch_count;
    }
  }

  ad9220_overrange_count = overrange_count;
  ad9220_glitch_correction_count += glitch_count;
  return count;
}

void AD9220_DMA_PortE_IRQHandler(void)
{
}

void AD9220_DMA_PortD_IRQHandler(void)
{
}

__weak void AD9220_CaptureCompleteCallback(void)
{
}

static HAL_StatusTypeDef AD9220_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 |
                         GPIO_PIN_14 | GPIO_PIN_15);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                         GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                         GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 |
             GPIO_PIN_14 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(GPIOD, &gpio);

  gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
             GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
             GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpio);
  return HAL_OK;
}

static void AD9220_CycleCounter_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
}

static uint16_t AD9220_PackSample(uint32_t gpio_snapshot)
{
  uint16_t port_e = (uint16_t)gpio_snapshot;
  uint16_t port_d =
      (uint16_t)(gpio_snapshot >> AD9220_PORT_D_SHIFT);
  uint16_t raw = 0U;

  raw |= (uint16_t)(((port_d >> 1U) & 1U) << 0U);
  raw |= (uint16_t)(((port_e >> 8U) & 1U) << 1U);
  raw |= (uint16_t)(((port_e >> 10U) & 1U) << 2U);
  raw |= (uint16_t)(((port_e >> 12U) & 1U) << 3U);
  raw |= (uint16_t)(((port_e >> 14U) & 1U) << 4U);
  raw |= (uint16_t)(((port_d >> 8U) & 1U) << 5U);
  raw |= (uint16_t)(((port_e >> 15U) & 1U) << 6U);
  raw |= (uint16_t)(((port_e >> 13U) & 1U) << 7U);
  raw |= (uint16_t)(((port_e >> 11U) & 1U) << 8U);
  raw |= (uint16_t)(((port_e >> 9U) & 1U) << 9U);
  raw |= (uint16_t)(((port_e >> 7U) & 1U) << 10U);
  raw |= (uint16_t)(((port_d >> 0U) & 1U) << 11U);
  return raw;
}

#include "ad9220.h"

#include "stm32h7xx_hal_dma_ex.h"

#include <stddef.h>

/*
 * AD9220 capture strategy used by the G-problem firmware
 * -------------------------------------------------------
 * AD9220 presents one 12-bit sample on pins split across GPIOE and GPIOD.
 * TIM4_CH4 supplies the continuous 8 MHz ADC clock.  The corresponding TIM4
 * update event drives two synchronized DMA streams, one per GPIO port, so
 * the CPU does not service an interrupt for every sample.  A completed pair
 * of DMA buffers is later packed into signed Q15 by AD9220_CopySignedSamples.
 */

#define AD9220_CAPTURE_TRANSFER_COUNT \
  (AD9220_MAX_CAPTURE_SAMPLES + AD9220_PIPELINE_DELAY)
#define AD9220_PORT_D_SHIFT               16U
#define AD9220_PORT_D_MASK(pin) \
  ((uint32_t)(pin) << AD9220_PORT_D_SHIFT)
#define AD9220_OTR_MASK                   AD9220_PORT_D_MASK(GPIO_PIN_14)
#define AD9220_CAPTURE_DONE_BUS           0x03U
#define AD9220_DMA_PORT_E_DONE            0x01U
#define AD9220_DMA_PORT_D_DONE            0x02U
#define AD9220_NO_READY_BUFFER             0xFFU
#define AD9220_TIM4_PRESCALER                 0U
#define AD9220_TIM4_KERNEL_CLOCK_HZ    240000000U
#define AD9220_TIM4_PERIOD \
  ((AD9220_TIM4_KERNEL_CLOCK_HZ / AD9220_SAMPLE_RATE_HZ) - 1U)
#define AD9220_TIM4_PULSE \
  ((AD9220_TIM4_PERIOD + 1U) / 2U)
#define AD9220_DMA_IRQ_PRIORITY                6U
#define AD9220_CAPTURE_ERROR_DMA          0x80000000U
#define AD9220_GLITCH_JUMP_THRESHOLD     8192L
#define AD9220_GLITCH_NEIGHBOR_THRESHOLD 8192L
#define AD9220_DMA_RAM \
  __attribute__((section(".ad9220_dma_ram"), aligned(32)))

/*
 * DMA1 cannot access DTCM. Each raw buffer therefore lives in D2 SRAM and
 * starts on its own cache-line boundary. TIM4 update requests make both DMA
 * streams read GPIOE/GPIOD at the configured sample rate. While DMA fills one
 * buffer, the CPU converts the other one.
 */
static uint32_t ad9220_port_e_buffer0[AD9220_CAPTURE_TRANSFER_COUNT]
    AD9220_DMA_RAM;
static uint32_t ad9220_port_e_buffer1[AD9220_CAPTURE_TRANSFER_COUNT]
    AD9220_DMA_RAM;
static uint32_t ad9220_port_d_buffer0[AD9220_CAPTURE_TRANSFER_COUNT]
    AD9220_DMA_RAM;
static uint32_t ad9220_port_d_buffer1[AD9220_CAPTURE_TRANSFER_COUNT]
    AD9220_DMA_RAM;

static DMA_HandleTypeDef ad9220_dma_port_e;
static DMA_HandleTypeDef ad9220_dma_port_d;
static TIM_HandleTypeDef ad9220_sample_timer;

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
static volatile uint32_t ad9220_buffer_generation[2];
static volatile uint32_t ad9220_ready_generation;
static volatile uint8_t ad9220_buffer_done_mask[2];
static volatile uint8_t ad9220_done_mask;
static volatile uint8_t ad9220_ready_buffer = AD9220_NO_READY_BUFFER;
static volatile uint8_t ad9220_capture_busy;
static volatile uint8_t ad9220_capture_complete;
static volatile uint8_t ad9220_fault_latched;
static uint8_t ad9220_initialized;

static HAL_StatusTypeDef AD9220_GPIO_Init(void);
static HAL_StatusTypeDef AD9220_Timer_Init(void);
static HAL_StatusTypeDef AD9220_DMA_Init(void);
static HAL_StatusTypeDef AD9220_DMA_Start(void);
static void AD9220_DMA_Stop(void);
static void AD9220_DMA_BufferComplete(uint8_t buffer_index,
                                      uint8_t port_mask);
static void AD9220_DMA_Error(DMA_HandleTypeDef *hdma);
static void AD9220_DMA_PortE_Buffer0Complete(DMA_HandleTypeDef *hdma);
static void AD9220_DMA_PortE_Buffer1Complete(DMA_HandleTypeDef *hdma);
static void AD9220_DMA_PortD_Buffer0Complete(DMA_HandleTypeDef *hdma);
static void AD9220_DMA_PortD_Buffer1Complete(DMA_HandleTypeDef *hdma);
static uint8_t AD9220_DMA_IsWritingBuffer(uint8_t buffer_index);
static void AD9220_InvalidateBuffer(uint32_t *buffer);
static uint32_t *AD9220_PortEBuffer(uint8_t buffer_index);
static uint32_t *AD9220_PortDBuffer(uint8_t buffer_index);
static uint16_t AD9220_PackSample(uint32_t gpio_snapshot);

void AD9220_Init(void)
{
  if (ad9220_initialized != 0U)
  {
    return;
  }

  ad9220_capture_busy = 0U;
  ad9220_capture_complete = 0U;
  ad9220_sample_rate_hz = 0U;
  ad9220_fault_latched = 0U;
  ad9220_done_mask = 0U;
  ad9220_ready_buffer = AD9220_NO_READY_BUFFER;
  ad9220_buffer_done_mask[0] = 0U;
  ad9220_buffer_done_mask[1] = 0U;
  ad9220_port_e_error_code = 0U;
  ad9220_port_d_error_code = 0U;
  ad9220_last_progress = 0U;
  ad9220_last_error_stage = 0U;

  if (AD9220_GPIO_Init() != HAL_OK)
  {
    ad9220_last_error_stage = 1U;
    ++ad9220_capture_error_count;
    return;
  }
  if (AD9220_Timer_Init() != HAL_OK)
  {
    ad9220_last_error_stage = 2U;
    ++ad9220_capture_error_count;
    return;
  }
  if (AD9220_DMA_Init() != HAL_OK)
  {
    ad9220_last_error_stage = 3U;
    ++ad9220_capture_error_count;
    return;
  }

  /*
   * Keep the ADC clock running continuously. Acquisition blocks only gate
   * the TIM4 update DMA request; the PWM phase and ADC pipeline never restart.
   */
  __HAL_TIM_SET_COUNTER(&ad9220_sample_timer, 0U);
  __HAL_TIM_CLEAR_FLAG(&ad9220_sample_timer, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE(&ad9220_sample_timer);

  ad9220_sample_rate_hz = AD9220_SAMPLE_RATE_HZ;
  ad9220_last_timer_delta = AD9220_TIM4_PERIOD + 1U;
  ad9220_timer_delta_limit = AD9220_TIM4_PERIOD + 1U;
  ad9220_initialized = 1U;
}

void AD9220_DeInit(void)
{
  AD9220_DMA_Stop();
  ad9220_capture_busy = 0U;
  ad9220_capture_complete = 0U;
  ad9220_sample_rate_hz = 0U;
  ad9220_done_mask = 0U;
  ad9220_ready_buffer = AD9220_NO_READY_BUFFER;
  ad9220_initialized = 0U;
}

AD9220_Status AD9220_StartCapture(uint32_t sample_count)
{
  uint32_t saved_primask;

  if ((sample_count == 0U) ||
      (sample_count > AD9220_MAX_CAPTURE_SAMPLES))
  {
    return AD9220_STATUS_INVALID_ARGUMENT;
  }
  if ((ad9220_initialized == 0U) ||
      (ad9220_fault_latched != 0U))
  {
    return AD9220_STATUS_ERROR;
  }

  saved_primask = __get_PRIMASK();
  __disable_irq();
  if (ad9220_capture_busy != 0U)
  {
    if (saved_primask == 0U)
    {
      __enable_irq();
    }
    return AD9220_STATUS_BUSY;
  }

  ad9220_requested_samples = sample_count;
  ad9220_captured_samples = 0U;
  ad9220_overrange_count = 0U;
  ad9220_done_mask = 0U;
  ad9220_capture_complete = 0U;
  ad9220_ready_buffer = AD9220_NO_READY_BUFFER;
  ad9220_capture_busy = 1U;
  ad9220_buffer_done_mask[0] = 0U;
  ad9220_buffer_done_mask[1] = 0U;
  ad9220_last_progress = 0U;
  ad9220_last_clock_level =
      ((GPIOD->IDR & AD9220_CLK_Pin) != 0U) ? 1U : 0U;
  __DMB();

  if (saved_primask == 0U)
  {
    __enable_irq();
  }

  if (AD9220_DMA_Start() != HAL_OK)
  {
    saved_primask = __get_PRIMASK();
    __disable_irq();
    ad9220_capture_busy = 0U;
    ad9220_capture_complete = 0U;
    ad9220_last_error_stage = 4U;
    ++ad9220_capture_error_count;
    __DMB();
    if (saved_primask == 0U)
    {
      __enable_irq();
    }
    return AD9220_STATUS_ERROR;
  }
  return AD9220_STATUS_OK;
}

void AD9220_AbortCapture(void)
{
  uint32_t saved_primask = __get_PRIMASK();

  __disable_irq();
  __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
  ad9220_capture_busy = 0U;
  ad9220_capture_complete = 0U;
  ad9220_done_mask = 0U;
  ad9220_ready_buffer = AD9220_NO_READY_BUFFER;
  __DMB();
  if (saved_primask == 0U)
  {
    __enable_irq();
  }
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
  uint8_t buffer_index = ad9220_ready_buffer;
  uint32_t source_index;
  uint32_t gpio_snapshot;

  if ((ad9220_capture_complete == 0U) ||
      (buffer_index > 1U) ||
      (index >= ad9220_captured_samples) ||
      (AD9220_DMA_IsWritingBuffer(buffer_index) != 0U))
  {
    return 0U;
  }

  source_index = index + AD9220_PIPELINE_DELAY;
  gpio_snapshot =
      (AD9220_PortDBuffer(buffer_index)[source_index] <<
       AD9220_PORT_D_SHIFT) |
      (AD9220_PortEBuffer(buffer_index)[source_index] & 0xFFFFU);
  return AD9220_PackSample(gpio_snapshot);
}

int16_t AD9220_GetSignedSample(uint32_t index)
{
  int32_t centered = (int32_t)AD9220_GetRawSample(index) - 2048;
  return (int16_t)(centered << 4);
}

uint8_t AD9220_GetOverrange(uint32_t index)
{
  uint8_t buffer_index = ad9220_ready_buffer;
  uint32_t source_index;

  if ((ad9220_capture_complete == 0U) ||
      (buffer_index > 1U) ||
      (index >= ad9220_captured_samples) ||
      (AD9220_DMA_IsWritingBuffer(buffer_index) != 0U))
  {
    return 0U;
  }

  source_index = index + AD9220_PIPELINE_DELAY;
  return ((AD9220_PortDBuffer(buffer_index)[source_index] &
           GPIO_PIN_14) != 0U) ? 1U : 0U;
}

__attribute__((optimize("O3")))
uint32_t AD9220_CopySignedSamples(int16_t *destination,
                                  uint32_t capacity)
{
  uint8_t buffer_index;
  uint32_t generation;
  uint32_t count;
  uint32_t overrange_count = 0U;
  uint32_t glitch_count = 0U;
  uint32_t *port_e;
  uint32_t *port_d;

  /*
   * Return zero rather than a partial block.  Task0729 requires exactly
   * 16384 contiguous samples, and accepting a half-written block would cause
   * much larger frequency/amplitude errors than dropping one result frame.
   */
  if ((destination == NULL) || (ad9220_capture_complete == 0U))
  {
    return 0U;
  }

  buffer_index = ad9220_ready_buffer;
  generation = ad9220_ready_generation;
  if ((buffer_index > 1U) ||
      (AD9220_DMA_IsWritingBuffer(buffer_index) != 0U))
  {
    ad9220_last_error_stage = 5U;
    return 0U;
  }

  port_e = AD9220_PortEBuffer(buffer_index);
  port_d = AD9220_PortDBuffer(buffer_index);
  AD9220_InvalidateBuffer(port_e);
  AD9220_InvalidateBuffer(port_d);

  count = ad9220_captured_samples;
  if (count > capacity)
  {
    count = capacity;
  }

  for (uint32_t i = 0U; i < count; ++i)
  {
    /* 先跳过AD9220流水线延迟，再拼出12位并行数据。 */
    uint32_t source_index = i + AD9220_PIPELINE_DELAY;
    uint32_t gpio_snapshot =
        (port_d[source_index] << AD9220_PORT_D_SHIFT) |
        (port_e[source_index] & 0xFFFFU);
    uint16_t raw = AD9220_PackSample(gpio_snapshot);

    /* 2048去中点；乘16把12位有符号码放大成FFT需要的Q15。 */
    destination[i] = (int16_t)(((int32_t)raw - 2048) << 4);
    if ((gpio_snapshot & AD9220_OTR_MASK) != 0U)
    {
      ++overrange_count;
    }
  }

  /*
   * Keep the isolated-glitch repair as a diagnostic safety net. With both
   * GPIO ports captured from the same timer request, GLITCH_FIX should fall
   * substantially compared with the split-port polling implementation.
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
        ((port_d[source_index] & GPIO_PIN_14) == 0U))
    {
      destination[i] = (int16_t)((previous + next) / 2L);
      ++glitch_count;
    }
  }

  /*
   * A buffer is valid only if DMA did not switch back to it while it was
   * being copied. This prevents a delayed task from accepting a mixed block.
   */
  __DMB();
  if ((ad9220_buffer_generation[buffer_index] != generation) ||
      (AD9220_DMA_IsWritingBuffer(buffer_index) != 0U))
  {
    ad9220_last_error_stage = 6U;
    return 0U;
  }

  ad9220_overrange_count = overrange_count;
  ad9220_glitch_correction_count += glitch_count;
  return count;
}

void AD9220_DMA_PortE_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&ad9220_dma_port_e);
}

void AD9220_DMA_PortD_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&ad9220_dma_port_d);
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
             GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(GPIOD, &gpio);

  gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
             GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
             GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpio);

  /*
   * TIM4_CH4 drives the AD9220 sampling clock directly on PD15. The inverted
   * PWM is low at counter update and rises at CCR4, so the TIM4 update DMA
   * request occurs half a clock after the ADC rising sampling edge.
   */
  gpio.Pin = AD9220_CLK_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(AD9220_CLK_GPIO_Port, &gpio);
  return HAL_OK;
}

static HAL_StatusTypeDef AD9220_Timer_Init(void)
{
  TIM_OC_InitTypeDef clock_channel = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  ad9220_sample_timer.Instance = TIM4;
  ad9220_sample_timer.Init.Prescaler = AD9220_TIM4_PRESCALER;
  ad9220_sample_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  ad9220_sample_timer.Init.Period = AD9220_TIM4_PERIOD;
  ad9220_sample_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  ad9220_sample_timer.Init.AutoReloadPreload =
      TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&ad9220_sample_timer) != HAL_OK)
  {
    return HAL_ERROR;
  }

  clock_channel.OCMode = TIM_OCMODE_PWM1;
  clock_channel.Pulse = AD9220_TIM4_PULSE;
  clock_channel.OCPolarity = TIM_OCPOLARITY_LOW;
  clock_channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  clock_channel.OCFastMode = TIM_OCFAST_DISABLE;
  clock_channel.OCIdleState = TIM_OCIDLESTATE_RESET;
  clock_channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&ad9220_sample_timer,
                                &clock_channel,
                                TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_DISABLE(&ad9220_sample_timer);
  __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
  __HAL_TIM_SET_COUNTER(&ad9220_sample_timer, 0U);
  __HAL_TIM_CLEAR_FLAG(&ad9220_sample_timer, TIM_FLAG_UPDATE);
  TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_4, TIM_CCx_ENABLE);
  return HAL_OK;
}

static HAL_StatusTypeDef AD9220_DMA_Init(void)
{
  HAL_DMA_MuxSyncConfigTypeDef event_config = {0};
  HAL_DMA_MuxRequestGeneratorConfigTypeDef generator_config = {0};

  __HAL_RCC_DMA1_CLK_ENABLE();

  ad9220_dma_port_e.Instance = DMA1_Stream0;
  ad9220_dma_port_e.Init.Request = DMA_REQUEST_TIM4_UP;
  ad9220_dma_port_e.Init.Direction = DMA_PERIPH_TO_MEMORY;
  ad9220_dma_port_e.Init.PeriphInc = DMA_PINC_DISABLE;
  ad9220_dma_port_e.Init.MemInc = DMA_MINC_ENABLE;
  ad9220_dma_port_e.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  ad9220_dma_port_e.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  ad9220_dma_port_e.Init.Mode = DMA_CIRCULAR;
  ad9220_dma_port_e.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  ad9220_dma_port_e.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  ad9220_dma_port_e.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  ad9220_dma_port_e.Init.MemBurst = DMA_MBURST_SINGLE;
  ad9220_dma_port_e.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&ad9220_dma_port_e) != HAL_OK)
  {
    ad9220_port_e_error_code = ad9220_dma_port_e.ErrorCode;
    return HAL_ERROR;
  }

  ad9220_dma_port_d.Instance = DMA1_Stream1;
  /*
   * DMAMUX1 does not permit two channels to select TIM4_UP directly. Stream0
   * generates a DMAMUX event after each TIM4 request; request generator 0
   * turns that event into the request consumed by Stream1.
   */
  ad9220_dma_port_d.Init.Request = DMA_REQUEST_GENERATOR0;
  ad9220_dma_port_d.Init.Direction = DMA_PERIPH_TO_MEMORY;
  ad9220_dma_port_d.Init.PeriphInc = DMA_PINC_DISABLE;
  ad9220_dma_port_d.Init.MemInc = DMA_MINC_ENABLE;
  ad9220_dma_port_d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  ad9220_dma_port_d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  ad9220_dma_port_d.Init.Mode = DMA_CIRCULAR;
  ad9220_dma_port_d.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  ad9220_dma_port_d.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  ad9220_dma_port_d.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  ad9220_dma_port_d.Init.MemBurst = DMA_MBURST_SINGLE;
  ad9220_dma_port_d.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&ad9220_dma_port_d) != HAL_OK)
  {
    ad9220_port_d_error_code = ad9220_dma_port_d.ErrorCode;
    (void)HAL_DMA_DeInit(&ad9220_dma_port_e);
    return HAL_ERROR;
  }

  ad9220_dma_port_e.XferCpltCallback =
      AD9220_DMA_PortE_Buffer0Complete;
  ad9220_dma_port_e.XferM1CpltCallback =
      AD9220_DMA_PortE_Buffer1Complete;
  ad9220_dma_port_e.XferErrorCallback = AD9220_DMA_Error;
  ad9220_dma_port_d.XferCpltCallback =
      AD9220_DMA_PortD_Buffer0Complete;
  ad9220_dma_port_d.XferM1CpltCallback =
      AD9220_DMA_PortD_Buffer1Complete;
  ad9220_dma_port_d.XferErrorCallback = AD9220_DMA_Error;

  /*
   * Enable an event on every request served by Stream0. With RequestNumber=1
   * the DMAMUX request counter reloads after every GPIOE transfer.
   */
  event_config.SyncSignalID = HAL_DMAMUX1_SYNC_DMAMUX1_CH0_EVT;
  event_config.SyncPolarity = HAL_DMAMUX_SYNC_NO_EVENT;
  event_config.SyncEnable = DISABLE;
  event_config.EventEnable = ENABLE;
  event_config.RequestNumber = 1U;
  if (HAL_DMAEx_ConfigMuxSync(&ad9220_dma_port_e,
                              &event_config) != HAL_OK)
  {
    ad9220_port_e_error_code = ad9220_dma_port_e.ErrorCode;
    return HAL_ERROR;
  }

  generator_config.SignalID = HAL_DMAMUX1_REQ_GEN_DMAMUX1_CH0_EVT;
  generator_config.Polarity = HAL_DMAMUX_REQ_GEN_RISING;
  generator_config.RequestNumber = 1U;
  if (HAL_DMAEx_ConfigMuxRequestGenerator(&ad9220_dma_port_d,
                                          &generator_config) != HAL_OK)
  {
    ad9220_port_d_error_code = ad9220_dma_port_d.ErrorCode;
    return HAL_ERROR;
  }
  if (HAL_DMAEx_EnableMuxRequestGenerator(&ad9220_dma_port_d) != HAL_OK)
  {
    ad9220_port_d_error_code = ad9220_dma_port_d.ErrorCode;
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, AD9220_DMA_IRQ_PRIORITY, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, AD9220_DMA_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  return HAL_OK;
}

static HAL_StatusTypeDef AD9220_DMA_Start(void)
{
  /*
   * A previous circular transfer may already have written a few words into
   * its next target before the completion ISR stopped TIM4. Abort and reload
   * both streams so every acquisition starts at index zero on both GPIO
   * ports, with no stale prefix from the preceding block.
   */
  __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
  if (ad9220_dma_port_e.State == HAL_DMA_STATE_BUSY)
  {
    if (HAL_DMA_Abort(&ad9220_dma_port_e) != HAL_OK)
    {
      ad9220_port_e_error_code = ad9220_dma_port_e.ErrorCode;
      return HAL_ERROR;
    }
  }
  if (ad9220_dma_port_d.State == HAL_DMA_STATE_BUSY)
  {
    if (HAL_DMA_Abort(&ad9220_dma_port_d) != HAL_OK)
    {
      ad9220_port_d_error_code = ad9220_dma_port_d.ErrorCode;
      return HAL_ERROR;
    }
  }

  CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_CT);
  CLEAR_BIT(DMA1_Stream1->CR, DMA_SxCR_CT);

  if (HAL_DMAEx_MultiBufferStart_IT(
          &ad9220_dma_port_e,
          (uint32_t)&GPIOE->IDR,
          (uint32_t)ad9220_port_e_buffer0,
          (uint32_t)ad9220_port_e_buffer1,
          AD9220_CAPTURE_TRANSFER_COUNT) != HAL_OK)
  {
    ad9220_port_e_error_code = ad9220_dma_port_e.ErrorCode;
    return HAL_ERROR;
  }

  if (HAL_DMAEx_MultiBufferStart_IT(
          &ad9220_dma_port_d,
          (uint32_t)&GPIOD->IDR,
          (uint32_t)ad9220_port_d_buffer0,
          (uint32_t)ad9220_port_d_buffer1,
          AD9220_CAPTURE_TRANSFER_COUNT) != HAL_OK)
  {
    ad9220_port_d_error_code = ad9220_dma_port_d.ErrorCode;
    (void)HAL_DMA_Abort(&ad9220_dma_port_e);
    return HAL_ERROR;
  }

  /*
   * TIM4 is the sole timing source: CH4 continuously outputs the ADC clock
   * and update events trigger both GPIO DMA streams. With inverted PWM,
   * update occurs on the PD15 falling edge, half a period after the ADC
   * rising edge.
   */
  __HAL_TIM_CLEAR_FLAG(&ad9220_sample_timer, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
  return HAL_OK;
}

static void AD9220_DMA_Stop(void)
{
  if (ad9220_sample_timer.Instance == TIM4)
  {
    __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE(&ad9220_sample_timer);
    TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_4, TIM_CCx_DISABLE);
  }

  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);

  if (ad9220_dma_port_d.DMAmuxRequestGen != 0U)
  {
    (void)HAL_DMAEx_DisableMuxRequestGenerator(&ad9220_dma_port_d);
  }

  if (ad9220_dma_port_e.Instance == DMA1_Stream0)
  {
    if (ad9220_dma_port_e.State == HAL_DMA_STATE_BUSY)
    {
      (void)HAL_DMA_Abort(&ad9220_dma_port_e);
    }
    (void)HAL_DMA_DeInit(&ad9220_dma_port_e);
  }
  if (ad9220_dma_port_d.Instance == DMA1_Stream1)
  {
    if (ad9220_dma_port_d.State == HAL_DMA_STATE_BUSY)
    {
      (void)HAL_DMA_Abort(&ad9220_dma_port_d);
    }
    (void)HAL_DMA_DeInit(&ad9220_dma_port_d);
  }

  if (ad9220_sample_timer.Instance == TIM4)
  {
    (void)HAL_TIM_PWM_DeInit(&ad9220_sample_timer);
  }
}

static void AD9220_DMA_BufferComplete(uint8_t buffer_index,
                                      uint8_t port_mask)
{
  uint8_t mask;

  __DMB();
  mask = ad9220_buffer_done_mask[buffer_index] | port_mask;
  ad9220_buffer_done_mask[buffer_index] = mask;
  if (ad9220_capture_busy != 0U)
  {
    ad9220_done_mask = mask;
  }

  if (mask != AD9220_CAPTURE_DONE_BUS)
  {
    return;
  }

  ad9220_buffer_done_mask[buffer_index] = 0U;
  ++ad9220_buffer_generation[buffer_index];
  if (ad9220_capture_busy == 0U)
  {
    return;
  }

  ad9220_ready_buffer = buffer_index;
  ad9220_ready_generation =
      ad9220_buffer_generation[buffer_index];
  ad9220_captured_samples = ad9220_requested_samples;
  ad9220_last_progress = AD9220_CAPTURE_TRANSFER_COUNT;
  ad9220_last_clock_level =
      ((GPIOD->IDR & AD9220_CLK_Pin) != 0U) ? 1U : 0U;
  ad9220_done_mask = AD9220_CAPTURE_DONE_BUS;
  ad9220_capture_complete = 1U;
  ad9220_capture_busy = 0U;

  /*
   * Freeze the completed DMA block before waking the main loop. TIM4_CH4
   * keeps driving the ADC continuously; the next capture aborts and reloads
   * both DMA streams before re-enabling update requests.
   */
  __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);
  __DMB();
  AD9220_CaptureCompleteCallback();
}

static void AD9220_DMA_Error(DMA_HandleTypeDef *hdma)
{
  if (hdma == &ad9220_dma_port_e)
  {
    ad9220_port_e_error_code =
        hdma->ErrorCode | AD9220_CAPTURE_ERROR_DMA;
    ad9220_last_error_stage = 7U;
  }
  else
  {
    ad9220_port_d_error_code =
        hdma->ErrorCode | AD9220_CAPTURE_ERROR_DMA;
    ad9220_last_error_stage = 8U;
  }

  ++ad9220_capture_error_count;
  ad9220_fault_latched = 1U;
  __HAL_TIM_DISABLE_DMA(&ad9220_sample_timer, TIM_DMA_UPDATE);

  if (ad9220_capture_busy != 0U)
  {
    ad9220_capture_busy = 0U;
    ad9220_capture_complete = 0U;
    AD9220_CaptureCompleteCallback();
  }
}

static void AD9220_DMA_PortE_Buffer0Complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  AD9220_DMA_BufferComplete(0U, AD9220_DMA_PORT_E_DONE);
}

static void AD9220_DMA_PortE_Buffer1Complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  AD9220_DMA_BufferComplete(1U, AD9220_DMA_PORT_E_DONE);
}

static void AD9220_DMA_PortD_Buffer0Complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  AD9220_DMA_BufferComplete(0U, AD9220_DMA_PORT_D_DONE);
}

static void AD9220_DMA_PortD_Buffer1Complete(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  AD9220_DMA_BufferComplete(1U, AD9220_DMA_PORT_D_DONE);
}

static uint8_t AD9220_DMA_IsWritingBuffer(uint8_t buffer_index)
{
  uint8_t port_e_target;
  uint8_t port_d_target;

  port_e_target =
      ((DMA1_Stream0->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;
  port_d_target =
      ((DMA1_Stream1->CR & DMA_SxCR_CT) != 0U) ? 1U : 0U;
  if (port_e_target != port_d_target)
  {
    return 1U;
  }
  return (port_e_target == buffer_index) ? 1U : 0U;
}

static void AD9220_InvalidateBuffer(uint32_t *buffer)
{
#if (__DCACHE_PRESENT == 1U)
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    int32_t size =
        (int32_t)((sizeof(ad9220_port_e_buffer0) + 31U) & ~31U);
    SCB_InvalidateDCache_by_Addr(buffer, size);
  }
#else
  (void)buffer;
#endif
}

static uint32_t *AD9220_PortEBuffer(uint8_t buffer_index)
{
  return (buffer_index == 0U) ?
         ad9220_port_e_buffer0 : ad9220_port_e_buffer1;
}

static uint32_t *AD9220_PortDBuffer(uint8_t buffer_index)
{
  return (buffer_index == 0U) ?
         ad9220_port_d_buffer0 : ad9220_port_d_buffer1;
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

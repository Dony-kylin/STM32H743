/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : scope_app.c
  * Description        : Bare-metal scope application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "scope_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9220.h"
#include "ad9220_spectrum.h"
#include "ad7606_scope.h"
#include "ad7606_scope_store.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  int16_t sample;
  uint16_t raw;
  uint8_t overrange;
  uint8_t reserved[3];
  uint32_t timestamp;
} AD9220_Frame;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AD9220_CAPTURE_TIMEOUT_MS     20U
#define AD9220_UART_BUFFER_SIZE       96U
#define AD9220_DUMP_BUFFER_SIZE       512U
#define AD9220_SPECTRUM_UART_BUFFER_SIZE 512U
#define AD9220_SPECTRUM_REQUEST_TIMEOUT_MS 3000U
#define AD9220_UART_MEASURE_PERIOD_MS 5000U
#define AD9220_FULL_SCALE_MV          2500U
#define AD9220_FULL_SCALE_100UV       25000L
#define AD9220_CAPTURE_RAM \
  __attribute__((section(".scope_ram"), aligned(32)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#if 0
osSemaphoreId_t AdcSemaphoreHandle;         /* ADC 转换完成信号量 */
osMessageQueueId_t AdcFrameQueueHandle;
volatile AD9220_Frame AdcLatestFrame;
volatile uint32_t AdcFrameCount;
volatile uint32_t AdcTimeoutCount;
volatile uint32_t AdcReadErrorCount;
volatile uint32_t AdcDmaDoneMask;
volatile uint32_t AdcTelemetryDropCount;
volatile uint32_t AdcUartErrorCount;
static volatile uint8_t AdcDumpRequest;
static volatile uint8_t AdcDumpReady;
static volatile uint32_t AdcDumpRequestTick;
static volatile uint8_t AdcAcquisitionEnabled;
static volatile uint8_t AdcCapturePauseRequest;
static volatile uint8_t AdcCapturePaused;
static volatile uint8_t AdcSpectrumEnabled = 1U;
static volatile uint8_t AdcSpectrumReady;
static volatile uint8_t AdcSpectrumFullRequest;
static volatile uint32_t AdcSpectrumFullRequestTick;
static volatile uint32_t AdcSpectrumErrorCount;
static volatile uint32_t AdcBadSampleCount;
static volatile uint8_t UartStreamEnabled = 1U;
static AD7606_ScopeConfig ScopeStartupConfig;
static uint32_t ScopeStartupSampleRateHz = AD9220_SAMPLE_RATE_HZ;
static uint8_t ScopeStartupStreamEnabled = 1U;
static uint8_t ScopeStartupConfigValid;
static uint8_t ScopeConfigDirty;
static int16_t AdcCaptureSamples[AD9220_CAPTURE_SAMPLES]
    AD9220_CAPTURE_RAM;
static AD9220_SpectrumResult AdcSpectrumResult;
static char UartStatusBuffer[512];

/* Task_Uart: CubeMX 不会自动生成，放在 USER CODE 区防止被清除 */
osThreadId_t Task_UartHandle;
const osThreadAttr_t Task_Uart_attributes = {
  .name = "Task_Uart",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE END Variables */
/* Definitions for Task_DataProces */
osThreadId_t Task_DataProcesHandle;
const osThreadAttr_t Task_DataProces_attributes = {
  .name = "Task_DataProces",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Lcd */
osThreadId_t Task_LcdHandle;
const osThreadAttr_t Task_Lcd_attributes = {
  .name = "Task_Lcd",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void AD9220_PrepareAcquisition(void);
static void UART_SendMeasurements(void);
static void UART_SendDump(void);
static void UART_SendSpectrumSummary(void);
static void UART_SendFullSpectrum(void);
static void UART_PollScopeCommands(void);
static void UART_HandleScopeCommand(char *command);
static void UART_SendText(const char *text);
static void UART_SendScopeStatus(void);
static size_t UART_StatusAppendText(size_t offset, const char *text);
static size_t UART_StatusAppendUnsigned(size_t offset, uint32_t value);
static size_t UART_StatusAppendSigned(size_t offset, int32_t value);
static size_t UART_StatusAppendHex(size_t offset, uint32_t value);
static uint8_t UART_SaveScopeConfig(void);
static void UART_Acknowledge(const char *ok_message);
static void AD9220_ApplyStartupScopeConfig(void);
void StartTask_Uart(void *argument);
void AD9220_CaptureCompleteCallback(void);

/* USER CODE END FunctionPrototypes */

void StartTask_DataProcess(void *argument);
void StartTask_Lcd(void *argument);

/* Legacy RTOS implementation retained below in an excluded block for reference. */

/**
  * @brief  Legacy RTOS initialization (not compiled)
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  AD7606_ScopeInit(AD9220_FULL_SCALE_MV);
  ScopeStartupConfigValid = AD7606_ScopeStoreLoad(
      &ScopeStartupConfig, &ScopeStartupSampleRateHz,
      &ScopeStartupStreamEnabled);
  ScopeConfigDirty = (ScopeStartupConfigValid == 0U) ? 1U : 0U;
  if (ScopeStartupConfigValid != 0U)
  {
    UartStreamEnabled = ScopeStartupStreamEnabled;
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 创建二进制信号量：初始计数为 0，最大计数为 1 */
  AdcSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  configASSERT(AdcSemaphoreHandle != NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  AdcFrameQueueHandle = osMessageQueueNew(AD9220_FRAME_QUEUE_DEPTH,
                                           sizeof(AD9220_Frame), NULL);
  configASSERT(AdcFrameQueueHandle != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_DataProces */
  Task_DataProcesHandle = osThreadNew(StartTask_DataProcess, NULL, &Task_DataProces_attributes);

  /* LCD is disabled to keep all SPI/CPU time available for acquisition. */
#if APP_LCD_ENABLED
  Task_LcdHandle = osThreadNew(StartTask_Lcd, NULL, &Task_Lcd_attributes);
  configASSERT(Task_LcdHandle != NULL);
#else
  Task_LcdHandle = NULL;
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Task_UartHandle = osThreadNew(StartTask_Uart, NULL, &Task_Uart_attributes);
  configASSERT(Task_UartHandle != NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_DataProcess */
/**
  * @brief  Function implementing the Task_DataProces thread.
  *         等待FMC总线DMA完成，并组装AD9220采样块。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_DataProcess */
void StartTask_DataProcess(void *argument)
{
  /* USER CODE BEGIN StartTask_DataProcess */
  uint32_t capture_block_count = 0U;

  (void)argument;
  AD9220_ApplyStartupScopeConfig();

  /* Let the UART task announce a successful boot before touching ADC pins. */
  while (AdcAcquisitionEnabled == 0U)
  {
    osDelay(1U);
  }
  AD9220_PrepareAcquisition();

  for(;;)
  {
    AD9220_Frame frame;
    AD9220_SpectrumResult spectrum_result;
    uint32_t copied;
    uint32_t bad_sample_count;
    uint8_t spectrum_published = 0U;

    if (AdcCapturePauseRequest != 0U)
    {
      AdcCapturePaused = 1U;
      osDelay(1U);
      continue;
    }
    AdcCapturePaused = 0U;

    while (osSemaphoreAcquire(AdcSemaphoreHandle, 0U) == osOK)
    {
    }

    if (AD9220_StartCapture(AD9220_CAPTURE_SAMPLES) !=
        AD9220_STATUS_OK)
    {
      ++AdcReadErrorCount;
      AD9220_PrepareAcquisition();
      osDelay(1U);
      continue;
    }

    if (osSemaphoreAcquire(AdcSemaphoreHandle,
                           AD9220_CAPTURE_TIMEOUT_TICKS) != osOK)
    {
      uint32_t capture_progress = AD9220_GetLastProgress();
      uint32_t dma_done_mask = AD9220_GetDmaDoneMask();

      ++AdcTimeoutCount;
      AdcDmaDoneMask = dma_done_mask;
      AD9220_AbortCapture();

      /*
       * A healthy 16384-point block completes in about 2.05 ms. If no DMA callback or
       * progress was observed during the timeout, restart the complete
       * TIM4/DMAMUX/DMA chain instead of repeatedly waiting on a dead engine.
       */
      if ((capture_progress == 0U) && (dma_done_mask == 0U))
      {
        ++AdcReadErrorCount;
        AD9220_PrepareAcquisition();
      }
      continue;
    }

    copied = AD9220_CopySignedSamples(
        AdcCaptureSamples, AD9220_CAPTURE_SAMPLES);
    if (copied != AD9220_CAPTURE_SAMPLES)
    {
      ++AdcReadErrorCount;
      AdcDmaDoneMask = AD9220_GetDmaDoneMask();
      AD9220_PrepareAcquisition();
      osDelay(1U);
      continue;
    }
    AdcDmaDoneMask = AD9220_GetDmaDoneMask();

    /*
     * Reject impossible near-full-scale samples before any time-domain or
     * frequency-domain measurement sees them. Isolated faults use the mean
     * of their valid neighbors; a consecutive run is linearly interpolated.
     */
    bad_sample_count = AD9220_SpectrumRepairSamples(
        AdcCaptureSamples, copied);
    AdcBadSampleCount += bad_sample_count;

    AD7606_ScopePushSamples(AdcCaptureSamples, copied,
                            AD9220_GetSampleRateHz());

    /*
     * Acquire one complete 16384-point block, then analyze that frozen block.
     * The next acquisition is not started until the UART task has consumed
     * this result, so FFT output can never refer to a buffer being overwritten.
     */
    if (AdcSpectrumEnabled != 0U)
    {
      if (AD9220_SpectrumAnalyze(AdcCaptureSamples, copied,
                                 AD9220_GetSampleRateHz(),
                                 bad_sample_count,
                                 &spectrum_result) != 0U)
      {
        taskENTER_CRITICAL();
        AdcSpectrumResult = spectrum_result;
        __DMB();
        AdcSpectrumReady = 1U;
        taskEXIT_CRITICAL();
        spectrum_published = 1U;
      }
      else
      {
        ++AdcSpectrumErrorCount;
      }
    }

    frame.sample = AdcCaptureSamples[copied - 1U];
    {
      int32_t raw = ((int32_t)frame.sample / 16L) + 2048L;
      if (raw < 0L)
      {
        raw = 0L;
      }
      else if (raw > 4095L)
      {
        raw = 4095L;
      }
      frame.raw = (uint16_t)raw;
    }
    frame.overrange =
        (AD9220_GetOverrangeCount() != 0U) ? 1U : 0U;
    frame.reserved[0] = 0U;
    frame.reserved[1] = 0U;
    frame.reserved[2] = 0U;
    frame.timestamp = HAL_GetTick();
    taskENTER_CRITICAL();
    AdcLatestFrame.sample = frame.sample;
    AdcLatestFrame.raw = frame.raw;
    AdcLatestFrame.overrange = frame.overrange;
    AdcLatestFrame.timestamp = frame.timestamp;
    AdcFrameCount += copied;
    taskEXIT_CRITICAL();

    /*
     * A DUMP uses this completed block in place. Pause before starting the
     * next capture so the UART task always sees one coherent waveform.
     */
    if (AdcDumpRequest != 0U)
    {
      taskENTER_CRITICAL();
      AdcDumpRequest = 0U;
      AdcDumpReady = 1U;
      taskEXIT_CRITICAL();

      while (AdcDumpReady != 0U)
      {
        osDelay(1U);
      }
    }

    if (spectrum_published != 0U)
    {
      while (AdcSpectrumReady != 0U)
      {
        osDelay(1U);
      }
    }

    ++capture_block_count;
    if ((capture_block_count % AD9220_UART_BLOCK_DECIMATION) == 0U)
    {
      if ((UartStreamEnabled != 0U) &&
          (osMessageQueuePut(AdcFrameQueueHandle, &frame,
                             0U, 0U) != osOK))
      {
        ++AdcTelemetryDropCount;
      }
      taskYIELD();  /* 让出 CPU 给 UART 发送数据，避免饥饿 */
    }

    /* 下一轮立即重新启动 DMA；外部 TCXO 的 8 MHz CLK 始终连续运行。 */
  }
  /* USER CODE END StartTask_DataProcess */
}

void AD9220_CaptureCompleteCallback(void)
{
  if (AdcSemaphoreHandle != NULL)
  {
    (void)osSemaphoreRelease(AdcSemaphoreHandle);
  }
}

/* USER CODE BEGIN Header_StartTask_Lcd */
/**
* @brief Function implementing the Task_Lcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Lcd */
void StartTask_Lcd(void *argument)
{
  /* USER CODE BEGIN StartTask_Lcd */
  (void)argument;
  SPI_LCD_Init();
  AD7606_ScopeDisplayInit();

  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);  /* LED 闪烁 */
    AD7606_ScopeDisplayRefresh();
    {
      uint32_t refresh_ms = AD7606_ScopeGetRefreshMs();
      if (refresh_ms < 250U)
      {
        refresh_ms = 250U;
      }
      osDelay(refresh_ms);
    }
  }
  /* USER CODE END StartTask_Lcd */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ======================== UART 遥测任务 ======================== */
void StartTask_Uart(void *argument)
{
  uint32_t last_measure_tick = 0U;
  AD9220_Frame frame;

  (void)argument;
  /* 等待 UART 外设和终端就绪 */
  osDelay(20U);

  UART_SendText(
      "#READY MODE=TIM_DMA_DB RATE=8000000Hz FFT=16384 CLEAN=2450mV\r\n");
  AdcAcquisitionEnabled = 1U;

  for(;;)
  {
    while (osMessageQueueGet(AdcFrameQueueHandle, &frame, NULL,
                             0U) == osOK)
    {
    }

    UART_PollScopeCommands();

    if ((AdcSpectrumFullRequest != 0U) &&
        ((HAL_GetTick() - AdcSpectrumFullRequestTick) >=
         AD9220_SPECTRUM_REQUEST_TIMEOUT_MS))
    {
      char error[96];

      AdcSpectrumFullRequest = 0U;
      (void)snprintf(
          error, sizeof(error),
          "#ERR SPECTRUM timeout stage=%lu fftErr=%lu samples=%lu\r\n",
          (unsigned long)AD9220_SpectrumGetStage(),
          (unsigned long)AdcSpectrumErrorCount,
          (unsigned long)AdcFrameCount);
      UART_SendText(error);
    }

    if ((AdcDumpRequest != 0U) &&
        ((HAL_GetTick() - AdcDumpRequestTick) >= 1000U))
    {
      taskENTER_CRITICAL();
      AdcDumpRequest = 0U;
      taskEXIT_CRITICAL();
      UART_SendText("#ERR DUMP acquisition timeout\r\n");
    }

    if (AdcDumpReady != 0U)
    {
      UART_SendDump();
      taskENTER_CRITICAL();
      AdcDumpReady = 0U;
      taskEXIT_CRITICAL();
      continue;
    }

    if (AdcSpectrumReady != 0U)
    {
      if (UartStreamEnabled != 0U)
      {
        UART_SendSpectrumSummary();
      }
      if (AdcSpectrumFullRequest != 0U)
      {
        UART_SendFullSpectrum();
        AdcSpectrumFullRequest = 0U;
      }
      taskENTER_CRITICAL();
      AdcSpectrumReady = 0U;
      taskEXIT_CRITICAL();
      continue;
    }

    {
      uint32_t now = HAL_GetTick();
      if ((UartStreamEnabled != 0U) &&
          (AdcSpectrumEnabled == 0U))
      {
        if ((now - last_measure_tick) >=
            AD9220_UART_MEASURE_PERIOD_MS)
        {
          last_measure_tick = now;
          UART_SendMeasurements();
        }
      }
    }

    /* 每 2 秒输出一次诊断 */
    osDelay(10U);
  }
}

/* ======================== AD9220 辅助函数 ======================== */

#endif

volatile AD9220_Frame AdcLatestFrame;
volatile uint32_t AdcFrameCount;
volatile uint32_t AdcTimeoutCount;
volatile uint32_t AdcReadErrorCount;
volatile uint32_t AdcDmaDoneMask;
volatile uint32_t AdcUartErrorCount;
static volatile uint8_t AdcCaptureDone;
static uint8_t AdcCaptureActive;
static uint32_t AdcCaptureStartTick;
static volatile uint8_t AdcDumpRequest;
static volatile uint8_t AdcDumpReady;
static volatile uint32_t AdcDumpRequestTick;
static volatile uint8_t AdcSpectrumEnabled = 1U;
static volatile uint8_t AdcSpectrumReady;
static volatile uint8_t AdcSpectrumFullRequest;
static volatile uint32_t AdcSpectrumFullRequestTick;
static volatile uint32_t AdcSpectrumErrorCount;
static volatile uint32_t AdcBadSampleCount;
static volatile uint8_t UartStreamEnabled = 1U;
static AD7606_ScopeConfig ScopeStartupConfig;
static uint32_t ScopeStartupSampleRateHz = AD9220_SAMPLE_RATE_HZ;
static uint8_t ScopeStartupStreamEnabled = 1U;
static uint8_t ScopeStartupConfigValid;
static uint8_t ScopeConfigDirty;
static int16_t AdcCaptureSamples[AD9220_CAPTURE_SAMPLES]
    AD9220_CAPTURE_RAM;
static AD9220_SpectrumResult AdcSpectrumResult;
static char UartStatusBuffer[512];
static uint32_t AdcLastMeasurementTick;
#if APP_LCD_ENABLED
static uint32_t ScopeLcdRefreshTick;
#endif

static void AD9220_PrepareAcquisition(void);
static void ScopeApp_ProcessAcquisition(void);
static void ScopeApp_ProcessCompletedCapture(void);
static void UART_SendMeasurements(void);
static void UART_SendDump(void);
static void UART_SendSpectrumSummary(void);
static void UART_SendFullSpectrum(void);
static void UART_PollScopeCommands(void);
static void UART_HandleScopeCommand(char *command);
static void UART_SendText(const char *text);
static void UART_SendScopeStatus(void);
static size_t UART_StatusAppendText(size_t offset, const char *text);
static size_t UART_StatusAppendUnsigned(size_t offset, uint32_t value);
static size_t UART_StatusAppendSigned(size_t offset, int32_t value);
static size_t UART_StatusAppendHex(size_t offset, uint32_t value);
static uint8_t UART_SaveScopeConfig(void);
static void UART_Acknowledge(const char *ok_message);
static void AD9220_ApplyStartupScopeConfig(void);

void ScopeApp_Init(void)
{
  AD7606_ScopeInit(AD9220_FULL_SCALE_MV);
  ScopeStartupConfigValid = AD7606_ScopeStoreLoad(
      &ScopeStartupConfig, &ScopeStartupSampleRateHz,
      &ScopeStartupStreamEnabled);
  ScopeConfigDirty = (ScopeStartupConfigValid == 0U) ? 1U : 0U;
  if (ScopeStartupConfigValid != 0U)
  {
    UartStreamEnabled = ScopeStartupStreamEnabled;
  }
  AD9220_ApplyStartupScopeConfig();

  HAL_Delay(20U);
  UART_SendText(
      "#READY MODE=TIM_DMA_DB RATE=8000000Hz FFT=16384 CLEAN=2450mV\r\n");

#if APP_LCD_ENABLED
  SPI_LCD_Init();
  AD7606_ScopeDisplayInit();
  ScopeLcdRefreshTick = HAL_GetTick();
#endif

  AD9220_PrepareAcquisition();
}

void ScopeApp_Process(void)
{
  uint32_t now;

  UART_PollScopeCommands();
  now = HAL_GetTick();

  if ((AdcSpectrumFullRequest != 0U) &&
      ((now - AdcSpectrumFullRequestTick) >=
       AD9220_SPECTRUM_REQUEST_TIMEOUT_MS))
  {
    char error[96];

    AdcSpectrumFullRequest = 0U;
    (void)snprintf(
        error, sizeof(error),
        "#ERR SPECTRUM timeout stage=%lu fftErr=%lu samples=%lu\r\n",
        (unsigned long)AD9220_SpectrumGetStage(),
        (unsigned long)AdcSpectrumErrorCount,
        (unsigned long)AdcFrameCount);
    UART_SendText(error);
  }

  if ((AdcDumpRequest != 0U) &&
      ((now - AdcDumpRequestTick) >= 1000U))
  {
    AdcDumpRequest = 0U;
    UART_SendText("#ERR DUMP acquisition timeout\r\n");
  }

  if (AdcDumpReady != 0U)
  {
    UART_SendDump();
    AdcDumpReady = 0U;
    return;
  }

  if (AdcSpectrumReady != 0U)
  {
    if (UartStreamEnabled != 0U)
    {
      UART_SendSpectrumSummary();
    }
    if (AdcSpectrumFullRequest != 0U)
    {
      UART_SendFullSpectrum();
      AdcSpectrumFullRequest = 0U;
    }
    AdcSpectrumReady = 0U;
    return;
  }

  if ((UartStreamEnabled != 0U) &&
      (AdcSpectrumEnabled == 0U) &&
      ((now - AdcLastMeasurementTick) >=
       AD9220_UART_MEASURE_PERIOD_MS))
  {
    AdcLastMeasurementTick = now;
    UART_SendMeasurements();
  }

#if APP_LCD_ENABLED
  if ((int32_t)(now - ScopeLcdRefreshTick) >= 0)
  {
    uint32_t refresh_ms = AD7606_ScopeGetRefreshMs();

    if (refresh_ms < 250U)
    {
      refresh_ms = 250U;
    }
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    AD7606_ScopeDisplayRefresh();
    ScopeLcdRefreshTick = now + refresh_ms;
  }
#endif

  ScopeApp_ProcessAcquisition();
}

static void ScopeApp_ProcessAcquisition(void)
{
  if (AdcCaptureActive != 0U)
  {
    if ((AdcCaptureDone != 0U) ||
        (AD9220_IsCaptureComplete() != 0U))
    {
      AdcCaptureActive = 0U;
      AdcCaptureDone = 0U;
      ScopeApp_ProcessCompletedCapture();
      return;
    }

    if ((HAL_GetTick() - AdcCaptureStartTick) >=
        AD9220_CAPTURE_TIMEOUT_MS)
    {
      uint32_t capture_progress = AD9220_GetLastProgress();
      uint32_t dma_done_mask = AD9220_GetDmaDoneMask();

      ++AdcTimeoutCount;
      AdcDmaDoneMask = dma_done_mask;
      AD9220_AbortCapture();
      AdcCaptureActive = 0U;
      AdcCaptureDone = 0U;
      if ((capture_progress == 0U) && (dma_done_mask == 0U))
      {
        ++AdcReadErrorCount;
      }
      AD9220_PrepareAcquisition();
    }
    return;
  }

  AdcCaptureDone = 0U;
  if (AD9220_StartCapture(AD9220_CAPTURE_SAMPLES) !=
      AD9220_STATUS_OK)
  {
    ++AdcReadErrorCount;
    AD9220_PrepareAcquisition();
    return;
  }
  AdcCaptureStartTick = HAL_GetTick();
  AdcCaptureActive = 1U;
}

static void ScopeApp_ProcessCompletedCapture(void)
{
  AD9220_Frame frame;
  AD9220_SpectrumResult spectrum_result;
  uint32_t copied;
  uint32_t bad_sample_count;

  copied = AD9220_CopySignedSamples(
      AdcCaptureSamples, AD9220_CAPTURE_SAMPLES);
  if (copied != AD9220_CAPTURE_SAMPLES)
  {
    ++AdcReadErrorCount;
    AdcDmaDoneMask = AD9220_GetDmaDoneMask();
    AD9220_PrepareAcquisition();
    return;
  }
  AdcDmaDoneMask = AD9220_GetDmaDoneMask();

  bad_sample_count = AD9220_SpectrumRepairSamples(
      AdcCaptureSamples, copied);
  AdcBadSampleCount += bad_sample_count;
  AD7606_ScopePushSamples(AdcCaptureSamples, copied,
                          AD9220_GetSampleRateHz());

  if (AdcSpectrumEnabled != 0U)
  {
    if (AD9220_SpectrumAnalyze(AdcCaptureSamples, copied,
                               AD9220_GetSampleRateHz(),
                               bad_sample_count,
                               &spectrum_result) != 0U)
    {
      AdcSpectrumResult = spectrum_result;
      __DMB();
      AdcSpectrumReady = 1U;
    }
    else
    {
      ++AdcSpectrumErrorCount;
    }
  }

  frame.sample = AdcCaptureSamples[copied - 1U];
  {
    int32_t raw = ((int32_t)frame.sample / 16L) + 2048L;
    if (raw < 0L)
    {
      raw = 0L;
    }
    else if (raw > 4095L)
    {
      raw = 4095L;
    }
    frame.raw = (uint16_t)raw;
  }
  frame.overrange =
      (AD9220_GetOverrangeCount() != 0U) ? 1U : 0U;
  frame.reserved[0] = 0U;
  frame.reserved[1] = 0U;
  frame.reserved[2] = 0U;
  frame.timestamp = HAL_GetTick();
  AdcLatestFrame.sample = frame.sample;
  AdcLatestFrame.raw = frame.raw;
  AdcLatestFrame.overrange = frame.overrange;
  AdcLatestFrame.timestamp = frame.timestamp;
  AdcFrameCount += copied;

  if (AdcDumpRequest != 0U)
  {
    AdcDumpRequest = 0U;
    AdcDumpReady = 1U;
  }
}

void AD9220_CaptureCompleteCallback(void)
{
  AdcCaptureDone = 1U;
}

static void UART_PollScopeCommands(void)
{
  static char command_buffer[64];
  static uint32_t command_length;
  uint8_t received;

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }

  while (HAL_UART_Receive(&huart1, &received, 1U, 0U) == HAL_OK)
  {
    if ((received == '\r') || (received == '\n'))
    {
      if (command_length != 0U)
      {
        command_buffer[command_length] = '\0';
        UART_HandleScopeCommand(command_buffer);
        command_length = 0U;
      }
    }
    else if ((received == '\b') || (received == 0x7FU))
    {
      if (command_length != 0U)
      {
        --command_length;
      }
    }
    else if ((received >= 0x20U) && (received <= 0x7EU))
    {
      if (command_length < (sizeof(command_buffer) - 1U))
      {
        command_buffer[command_length++] = (char)received;
      }
      else
      {
        command_length = 0U;
        UART_SendText("#ERR command too long\r\n");
      }
    }
  }
}

static void UART_HandleScopeCommand(char *command)
{
  char *cursor = command;
  char *end;
  char extra;
  long signed_value;
  unsigned long value;

  while ((*cursor != '\0') && (isspace((unsigned char)*cursor) != 0))
  {
    ++cursor;
  }
  end = cursor + strlen(cursor);
  while ((end > cursor) && (isspace((unsigned char)end[-1]) != 0))
  {
    *--end = '\0';
  }
  for (char *p = cursor; *p != '\0'; ++p)
  {
    *p = (char)toupper((unsigned char)*p);
  }

  if (strcmp(cursor, "HELP") == 0)
  {
    UART_SendText(
        "#CMD CH 1 | VDIV 50/100/200/500/1000/2000\r\n"
        "#CMD CENTER -2500..2500(mV) or CENTER AUTO\r\n"
        "#CMD RATE 8000000 (effective sample rate)\r\n"
        "#CMD TIME AUTO or TIME 10..1000000(us/div)\r\n"
        "#CMD DEC 1/2/4/8/16/32 | REFRESH 250..2000(ms)\r\n"
        "#CMD AUTO (fit VDIV/CENTER/TIME; keep DEC)\r\n"
        "#CMD DUMP (one 16384-point CSV waveform)\r\n"
        "#CMD FFT ON/OFF (16384-point H1..H10 and THD)\r\n"
        "#CMD SPECTRUM (one full 0..Nyquist FFT CSV)\r\n"
        "#CMD RUN | STOP | STREAM ON/OFF | SAVE | STATUS | HELP\r\n");
  }
  else if (strcmp(cursor, "STATUS") == 0)
  {
    UART_SendScopeStatus();
  }
  else if (strcmp(cursor, "SAVE") == 0)
  {
    if (UART_SaveScopeConfig() != 0U)
    {
      ScopeConfigDirty = 0U;
      UART_SendText("#OK SAVED TO FLASH\r\n");
    }
    else
    {
      UART_SendText("#ERR FLASH save failed\r\n");
    }
  }
  else if (strcmp(cursor, "DUMP") == 0)
  {
    if ((AdcDumpRequest != 0U) || (AdcDumpReady != 0U))
    {
      UART_SendText("#ERR DUMP BUSY\r\n");
    }
    else
    {
      AdcDumpRequest = 1U;
      AdcDumpRequestTick = HAL_GetTick();
      UART_SendText("#OK DUMP ARMED\r\n");
    }
  }
  else if (strcmp(cursor, "FFT ON") == 0)
  {
    AdcSpectrumEnabled = 1U;
    UART_SendText("#OK FFT ON\r\n");
  }
  else if (strcmp(cursor, "FFT OFF") == 0)
  {
    AdcSpectrumEnabled = 0U;
    AdcSpectrumFullRequest = 0U;
    UART_SendText("#OK FFT OFF\r\n");
  }
  else if ((strcmp(cursor, "SPECTRUM") == 0) ||
           (strcmp(cursor, "FFT FULL") == 0))
  {
    if (AdcSpectrumFullRequest != 0U)
    {
      UART_SendText("#ERR SPECTRUM BUSY\r\n");
    }
    else
    {
      AdcSpectrumEnabled = 1U;
      AdcSpectrumFullRequest = 1U;
      AdcSpectrumFullRequestTick = HAL_GetTick();
      UART_SendText("#OK SPECTRUM ARMED\r\n");
    }
  }
  else if (strcmp(cursor, "RUN") == 0)
  {
    AD7606_ScopeSetRunning(1U);
    UART_Acknowledge("#OK RUN\r\n");
  }
  else if (strcmp(cursor, "STOP") == 0)
  {
    AD7606_ScopeSetRunning(0U);
    UART_Acknowledge("#OK HOLD\r\n");
  }
  else if (strcmp(cursor, "STREAM ON") == 0)
  {
    UartStreamEnabled = 1U;
    UART_Acknowledge("#OK STREAM ON\r\n");
  }
  else if (strcmp(cursor, "STREAM OFF") == 0)
  {
    UartStreamEnabled = 0U;
    UART_Acknowledge("#OK STREAM OFF\r\n");
  }
  else if (strcmp(cursor, "TIME AUTO") == 0)
  {
    (void)AD7606_ScopeSetTimePerDivUs(0U);
    UART_Acknowledge("#OK TIME AUTO\r\n");
  }
  else if (strcmp(cursor, "CENTER AUTO") == 0)
  {
    AD7606_ScopeSetCenterAuto(1U);
    UART_Acknowledge("#OK CENTER AUTO\r\n");
  }
  else if (strcmp(cursor, "AUTO") == 0)
  {
    if (AD7606_ScopeAutoConfigure() != 0U)
    {
      UART_Acknowledge("#OK AUTO\r\n");
    }
    else
    {
      UART_SendText("#ERR AUTO waiting for sampled signal\r\n");
    }
  }
  else if (sscanf(cursor, "CH %lu %c", &value, &extra) == 1)
  {
    if (AD7606_ScopeSetChannel((uint32_t)value) != 0U)
    {
      UART_Acknowledge("#OK CH\r\n");
    }
    else
    {
      UART_SendText("#ERR CH must be 1\r\n");
    }
  }
  else if (sscanf(cursor, "VDIV %lu %c", &value, &extra) == 1)
  {
    if (AD7606_ScopeSetMvPerDiv((uint32_t)value) != 0U)
    {
      UART_Acknowledge("#OK VDIV\r\n");
    }
    else
    {
      UART_SendText("#ERR VDIV 50/100/200/500/1000/2000\r\n");
    }
  }
  else if (sscanf(cursor, "CENTER %ld %c", &signed_value, &extra) == 1)
  {
    if (AD7606_ScopeSetCenterMv((int32_t)signed_value) != 0U)
    {
      UART_Acknowledge("#OK CENTER\r\n");
    }
    else
    {
      UART_SendText("#ERR CENTER outside ADC range\r\n");
    }
  }
  else if (sscanf(cursor, "RATE %lu %c", &value, &extra) == 1)
  {
    if (value == (unsigned long)AD9220_GetSampleRateHz())
    {
      UART_Acknowledge("#OK RATE\r\n");
    }
    else
    {
      UART_SendText("#ERR RATE fixed at 8000000 sps\r\n");
    }
  }
  else if (sscanf(cursor, "TIME %lu %c", &value, &extra) == 1)
  {
    if (AD7606_ScopeSetTimePerDivUs((uint32_t)value) != 0U)
    {
      UART_Acknowledge("#OK TIME\r\n");
    }
    else
    {
      UART_SendText("#ERR TIME 10..1000000 us/div\r\n");
    }
  }
  else if (sscanf(cursor, "DEC %lu %c", &value, &extra) == 1)
  {
    if (AD7606_ScopeSetDecimation((uint32_t)value) != 0U)
    {
      UART_Acknowledge("#OK DEC\r\n");
    }
    else
    {
      UART_SendText("#ERR DEC 1/2/4/8/16/32\r\n");
    }
  }
  else if (sscanf(cursor, "REFRESH %lu %c", &value, &extra) == 1)
  {
    if (AD7606_ScopeSetRefreshMs((uint32_t)value) != 0U)
    {
      UART_Acknowledge("#OK REFRESH\r\n");
    }
    else
    {
      UART_SendText("#ERR REFRESH 250..2000 ms\r\n");
    }
  }
  else
  {
    UART_SendText("#ERR unknown command; type HELP\r\n");
  }
}

static void UART_SendText(const char *text)
{
  size_t length = strlen(text);
  if ((length > 0U) &&
      (HAL_UART_Transmit(&huart1, (const uint8_t *)text,
                         (uint16_t)length, 200U) != HAL_OK))
  {
    ++AdcUartErrorCount;
  }
}

static void UART_SendScopeStatus(void)
{
  AD7606_ScopeConfig config;
  uint32_t sample_rate_hz;
  uint32_t target_rate_hz;
  uint32_t overrun_count;
  uint32_t glitch_correction_count;
  uint32_t dma_error_count;
  uint32_t dma_error_code;
  uint32_t dma_done_mask;
  uint32_t timer_delta;
  uint32_t timer_delta_limit;
  uint32_t capture_progress;
  uint32_t error_stage;
  uint32_t clock_level;
  size_t offset = 0U;

  AD7606_ScopeGetConfig(&config);
  sample_rate_hz = AD7606_ScopeGetInputSampleRateHz();
  target_rate_hz = AD9220_GetSampleRateHz();
  overrun_count = AD9220_GetOverrangeCount();
  glitch_correction_count = AD9220_GetGlitchCorrectionCount();
  dma_error_count = AD9220_GetDmaErrorCount();
  dma_error_code = AD9220_GetPortEDmaErrorCode();
  dma_done_mask = AD9220_GetDmaDoneMask();
  timer_delta = AD9220_GetLastTimerDelta();
  timer_delta_limit = AD9220_GetTimerDeltaLimit();
  capture_progress = AD9220_GetLastProgress();
  error_stage = AD9220_GetLastErrorStage();
  clock_level = AD9220_GetLastClockLevel();

  offset = UART_StatusAppendText(offset, "#SCOPE ");
  offset = UART_StatusAppendText(
      offset, (config.running != 0U) ? "RUN CH=" : "HOLD CH=");
  offset = UART_StatusAppendUnsigned(offset, config.channel);
  offset = UART_StatusAppendText(offset, " VDIV=");
  offset = UART_StatusAppendUnsigned(offset, config.mv_per_div);
  offset = UART_StatusAppendText(offset, "mV CENTER=");
  if (config.center_auto != 0U)
  {
    offset = UART_StatusAppendText(offset, "AUTO");
  }
  else
  {
    if (config.center_mv >= 0)
    {
      offset = UART_StatusAppendText(offset, "+");
    }
    offset = UART_StatusAppendSigned(offset, config.center_mv);
    offset = UART_StatusAppendText(offset, "mV");
  }

  offset = UART_StatusAppendText(offset, " TIME=");
  if (config.time_per_div_us == 0U)
  {
    offset = UART_StatusAppendText(offset, "AUTO");
  }
  else
  {
    offset = UART_StatusAppendUnsigned(offset, config.time_per_div_us);
    offset = UART_StatusAppendText(offset, "us/div");
  }

  offset = UART_StatusAppendText(offset, " DEC=");
  offset = UART_StatusAppendUnsigned(offset, config.decimation);
  offset = UART_StatusAppendText(offset, " RATE=");
  offset = UART_StatusAppendUnsigned(offset, target_rate_hz);
  offset = UART_StatusAppendText(offset, "Hz FS=");
  offset = UART_StatusAppendUnsigned(offset, sample_rate_hz);
  offset = UART_StatusAppendText(offset, "Hz SAMPLES=");
  offset = UART_StatusAppendUnsigned(offset, AdcFrameCount);
  offset = UART_StatusAppendText(offset, " OTR=");
  offset = UART_StatusAppendUnsigned(offset, overrun_count);
  offset = UART_StatusAppendText(offset, " GLITCH_FIX=");
  offset = UART_StatusAppendUnsigned(offset, glitch_correction_count);
  offset = UART_StatusAppendText(offset, " CAP_ERR=");
  offset = UART_StatusAppendUnsigned(offset, AdcReadErrorCount);
  offset = UART_StatusAppendText(offset, " MODE=TIM_DMA_DB DRV_ERR=");
  offset = UART_StatusAppendUnsigned(offset, dma_error_count);
  offset = UART_StatusAppendText(offset, " DONE=");
  offset = UART_StatusAppendUnsigned(offset, dma_done_mask);
  offset = UART_StatusAppendText(offset, " ERR=0x");
  offset = UART_StatusAppendHex(offset, dma_error_code);
  offset = UART_StatusAppendText(offset, " STAGE=");
  offset = UART_StatusAppendUnsigned(offset, error_stage);
  offset = UART_StatusAppendText(offset, " PROG=");
  offset = UART_StatusAppendUnsigned(offset, capture_progress);
  offset = UART_StatusAppendText(offset, " CLK=");
  offset = UART_StatusAppendText(offset, (clock_level != 0U) ? "H" : "L");
  offset = UART_StatusAppendText(offset, " TIMEOUT=");
  offset = UART_StatusAppendUnsigned(offset, AdcTimeoutCount);
  offset = UART_StatusAppendText(offset, " DT=");
  offset = UART_StatusAppendUnsigned(offset, timer_delta);
  offset = UART_StatusAppendText(offset, "/");
  offset = UART_StatusAppendUnsigned(offset, timer_delta_limit);
  offset = UART_StatusAppendText(offset, " REFRESH=");
  offset = UART_StatusAppendUnsigned(offset, config.refresh_ms);
  offset = UART_StatusAppendText(offset, "ms STREAM=");
  offset = UART_StatusAppendText(
      offset, (UartStreamEnabled != 0U) ? "ON FFT=" : "OFF FFT=");
  offset = UART_StatusAppendText(
      offset, (AdcSpectrumEnabled != 0U) ? "ON BAD=" : "OFF BAD=");
  offset = UART_StatusAppendUnsigned(offset, AdcBadSampleCount);
  offset = UART_StatusAppendText(offset, " FFT_ERR=");
  offset = UART_StatusAppendUnsigned(offset, AdcSpectrumErrorCount);
  offset = UART_StatusAppendText(offset, " FFT_STAGE=");
  offset = UART_StatusAppendUnsigned(
      offset, AD9220_SpectrumGetStage());
  offset = UART_StatusAppendText(offset, " FFT_REQ=");
  offset = UART_StatusAppendUnsigned(
      offset, (uint32_t)AdcSpectrumFullRequest);
  offset = UART_StatusAppendText(offset, " CFG=");
  offset = UART_StatusAppendText(
      offset, (ScopeConfigDirty != 0U) ? "DIRTY\r\n" : "SAVED\r\n");
  UART_SendText(UartStatusBuffer);
}

static size_t UART_StatusAppendText(size_t offset, const char *text)
{
  while ((*text != '\0') &&
         ((offset + 1U) < sizeof(UartStatusBuffer)))
  {
    UartStatusBuffer[offset++] = *text++;
  }
  UartStatusBuffer[offset] = '\0';
  return offset;
}

static size_t UART_StatusAppendUnsigned(size_t offset, uint32_t value)
{
  char digits[10];
  uint32_t count = 0U;

  do
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (count < sizeof(digits)));

  while (count != 0U)
  {
    char character[2];
    character[0] = digits[--count];
    character[1] = '\0';
    offset = UART_StatusAppendText(offset, character);
  }
  return offset;
}

static size_t UART_StatusAppendSigned(size_t offset, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    offset = UART_StatusAppendText(offset, "-");
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }
  return UART_StatusAppendUnsigned(offset, magnitude);
}

static size_t UART_StatusAppendHex(size_t offset, uint32_t value)
{
  static const char hex_digits[] = "0123456789ABCDEF";
  char digits[8];
  uint32_t count = 0U;

  do
  {
    digits[count++] = hex_digits[value & 0xFU];
    value >>= 4U;
  } while ((value != 0U) && (count < sizeof(digits)));

  while (count != 0U)
  {
    char character[2];
    character[0] = digits[--count];
    character[1] = '\0';
    offset = UART_StatusAppendText(offset, character);
  }
  return offset;
}

static uint8_t UART_SaveScopeConfig(void)
{
  AD7606_ScopeConfig config;
  uint32_t sample_rate_hz = AD9220_GetSampleRateHz();

  AD7606_ScopeGetConfig(&config);
  if (sample_rate_hz == 0U)
  {
    sample_rate_hz = ScopeStartupSampleRateHz;
  }
  return AD7606_ScopeStoreSave(&config, sample_rate_hz,
                               UartStreamEnabled);
}

static void UART_Acknowledge(const char *ok_message)
{
  ScopeConfigDirty = 1U;
  UART_SendText(ok_message);
}

static void AD9220_ApplyStartupScopeConfig(void)
{
  if (ScopeStartupConfigValid == 0U)
  {
    return;
  }

  (void)AD7606_ScopeSetChannel(ScopeStartupConfig.channel);
  (void)AD7606_ScopeSetMvPerDiv(ScopeStartupConfig.mv_per_div);
  (void)AD7606_ScopeSetDecimation(ScopeStartupConfig.decimation);
  (void)AD7606_ScopeSetCenterMv(ScopeStartupConfig.center_mv);
  if (ScopeStartupConfig.center_auto != 0U)
  {
    AD7606_ScopeSetCenterAuto(1U);
  }
  (void)AD7606_ScopeSetTimePerDivUs(
      ScopeStartupConfig.time_per_div_us);
  (void)AD7606_ScopeSetRefreshMs(ScopeStartupConfig.refresh_ms);
  AD7606_ScopeSetRunning(ScopeStartupConfig.running);
}

static void AD9220_PrepareAcquisition(void)
{
  AD9220_DeInit();
  AD9220_Init();
  AdcCaptureDone = 0U;
  AdcCaptureActive = 0U;
}

static void UART_SendMeasurements(void)
{
  AD7606_ScopeMeasurements measurements;
  char buffer[AD9220_UART_BUFFER_SIZE];

  if (AD7606_ScopeGetMeasurements(&measurements) == 0U)
  {
    return;
  }

  if (measurements.frequency_millihz != 0U)
  {
    uint32_t frequency_tenth_hz =
        (measurements.frequency_millihz + 50U) / 100U;
    (void)snprintf(buffer, sizeof(buffer),
                   "F=%lu.%luHz A=%lu.%03luV\r\n",
                   (unsigned long)(frequency_tenth_hz / 10U),
                   (unsigned long)(frequency_tenth_hz % 10U),
                   (unsigned long)(measurements.amplitude_mv / 1000U),
                   (unsigned long)(measurements.amplitude_mv % 1000U));
  }
  else
  {
    (void)snprintf(buffer, sizeof(buffer),
                   "F=---Hz A=%lu.%03luV\r\n",
                   (unsigned long)(measurements.amplitude_mv / 1000U),
                   (unsigned long)(measurements.amplitude_mv % 1000U));
  }
  UART_SendText(buffer);
}

static void UART_SendDump(void)
{
  char buffer[AD9220_DUMP_BUFFER_SIZE];
  uint32_t used = 0U;

  (void)snprintf(buffer, sizeof(buffer),
                 "#DUMP BEGIN count=%lu fs=%luHz columns=voltage_V\r\n",
                 (unsigned long)AD9220_CAPTURE_SAMPLES,
                 (unsigned long)AD9220_GetSampleRateHz());
  UART_SendText(buffer);

  for (uint32_t i = 0U; i < AD9220_CAPTURE_SAMPLES; ++i)
  {
    int32_t scaled;
    uint32_t magnitude;
    const char *sign;
    int written;

    /*
     * Leave enough room for the longest CSV row before formatting. Sending
     * 512-byte chunks avoids the overhead of 16384 individual UART calls.
     */
    if (used > (sizeof(buffer) - 32U))
    {
      if (HAL_UART_Transmit(&huart1, (const uint8_t *)buffer,
                            (uint16_t)used, 200U) != HAL_OK)
      {
        ++AdcUartErrorCount;
        UART_SendText("#DUMP ABORT UART\r\n");
        return;
      }
      used = 0U;
    }

    scaled = (int32_t)(((int64_t)AdcCaptureSamples[i] *
                        AD9220_FULL_SCALE_100UV) / 32768L);
    magnitude = (scaled < 0) ?
                (uint32_t)(-scaled) : (uint32_t)scaled;
    sign = (scaled < 0) ? "-" : "";
    written = snprintf(&buffer[used], sizeof(buffer) - used,
                       "%s%lu.%04lu\r\n",
                       sign,
                       (unsigned long)(magnitude / 10000U),
                       (unsigned long)(magnitude % 10000U));
    if ((written < 0) ||
        ((uint32_t)written >= (sizeof(buffer) - used)))
    {
      ++AdcUartErrorCount;
      UART_SendText("#DUMP ABORT FORMAT\r\n");
      return;
    }
    used += (uint32_t)written;
  }

  if ((used != 0U) &&
      (HAL_UART_Transmit(&huart1, (const uint8_t *)buffer,
                         (uint16_t)used, 200U) != HAL_OK))
  {
    ++AdcUartErrorCount;
    UART_SendText("#DUMP ABORT UART\r\n");
    return;
  }
  UART_SendText("#DUMP END\r\n");
}

static void UART_SendSpectrumSummary(void)
{
  AD9220_SpectrumResult result = AdcSpectrumResult;
  char buffer[AD9220_SPECTRUM_UART_BUFFER_SIZE];
  uint32_t used = 0U;
  int written;

  if (result.valid == 0U)
  {
    UART_SendText("#FFT INVALID\r\n");
    return;
  }

  written = snprintf(
      buffer, sizeof(buffer),
      "#FFT seq=%lu n=%lu fs=%luHz df=%lu.%03luHz "
      "f0=%lu.%03luHz A1=%lu.%06luV THD=%lu.%04lu%% "
      "BAD=%lu T=%luus\r\n",
      (unsigned long)result.sequence,
      (unsigned long)result.fft_size,
      (unsigned long)result.sample_rate_hz,
      (unsigned long)(result.bin_width_millihz / 1000U),
      (unsigned long)(result.bin_width_millihz % 1000U),
      (unsigned long)(result.fundamental_millihz / 1000U),
      (unsigned long)(result.fundamental_millihz % 1000U),
      (unsigned long)(result.fundamental_amplitude_uv / 1000000U),
      (unsigned long)(result.fundamental_amplitude_uv % 1000000U),
      (unsigned long)(result.thd_ppm / 10000U),
      (unsigned long)(result.thd_ppm % 10000U),
      (unsigned long)result.bad_sample_count,
      (unsigned long)result.analysis_time_us);
  if ((written < 0) || ((uint32_t)written >= sizeof(buffer)))
  {
    UART_SendText("#FFT ABORT FORMAT\r\n");
    return;
  }
  used = (uint32_t)written;

  for (uint32_t harmonic = 0U;
       harmonic < AD9220_SPECTRUM_HARMONIC_COUNT; ++harmonic)
  {
    uint32_t frequency_millihz =
        result.harmonic_frequency_millihz[harmonic];
    uint32_t amplitude_uv =
        result.harmonic_amplitude_uv[harmonic];

    if (frequency_millihz == 0U)
    {
      break;
    }

    written = snprintf(
        &buffer[used], sizeof(buffer) - used,
        "H%lu=%lu.%03luHz/%lu.%06luV%s",
        (unsigned long)(harmonic + 1U),
        (unsigned long)(frequency_millihz / 1000U),
        (unsigned long)(frequency_millihz % 1000U),
        (unsigned long)(amplitude_uv / 1000000U),
        (unsigned long)(amplitude_uv % 1000000U),
        ((harmonic + 1U) == AD9220_SPECTRUM_HARMONIC_COUNT) ?
            "\r\n" : " ");
    if ((written < 0) ||
        ((uint32_t)written >= (sizeof(buffer) - used)))
    {
      UART_SendText("#FFT ABORT FORMAT\r\n");
      return;
    }
    used += (uint32_t)written;
  }

  if ((used < 2U) ||
      (buffer[used - 2U] != '\r') ||
      (buffer[used - 1U] != '\n'))
  {
    if ((used + 2U) >= sizeof(buffer))
    {
      UART_SendText("#FFT ABORT FORMAT\r\n");
      return;
    }
    buffer[used++] = '\r';
    buffer[used++] = '\n';
    buffer[used] = '\0';
  }
  UART_SendText(buffer);
}

static void UART_SendFullSpectrum(void)
{
  AD9220_SpectrumResult result = AdcSpectrumResult;
  char buffer[AD9220_DUMP_BUFFER_SIZE];
  uint32_t used = 0U;

  (void)snprintf(
      buffer, sizeof(buffer),
      "#SPECTRUM BEGIN seq=%lu bins=%lu fs=%luHz "
      "window=HANN dc=removed "
      "columns=bin,frequency_Hz,amplitude_Vpeak\r\n",
      (unsigned long)result.sequence,
      (unsigned long)AD9220_SPECTRUM_BIN_COUNT,
      (unsigned long)result.sample_rate_hz);
  UART_SendText(buffer);

  for (uint32_t bin = 0U;
       bin < AD9220_SPECTRUM_BIN_COUNT; ++bin)
  {
    uint64_t frequency_millihz =
        ((uint64_t)bin * result.sample_rate_hz * 1000U) /
        AD9220_SPECTRUM_FFT_SIZE;
    uint32_t amplitude_uv =
        AD9220_SpectrumGetBinMagnitudeUv(bin);
    int written;

    if (used > (sizeof(buffer) - 64U))
    {
      if (HAL_UART_Transmit(&huart1, (const uint8_t *)buffer,
                            (uint16_t)used, 200U) != HAL_OK)
      {
        ++AdcUartErrorCount;
        UART_SendText("#SPECTRUM ABORT UART\r\n");
        return;
      }
      used = 0U;
    }

    written = snprintf(
        &buffer[used], sizeof(buffer) - used,
        "%lu,%lu.%03lu,%lu.%06lu\r\n",
        (unsigned long)bin,
        (unsigned long)(frequency_millihz / 1000U),
        (unsigned long)(frequency_millihz % 1000U),
        (unsigned long)(amplitude_uv / 1000000U),
        (unsigned long)(amplitude_uv % 1000000U));
    if ((written < 0) ||
        ((uint32_t)written >= (sizeof(buffer) - used)))
    {
      ++AdcUartErrorCount;
      UART_SendText("#SPECTRUM ABORT FORMAT\r\n");
      return;
    }
    used += (uint32_t)written;
  }

  if ((used != 0U) &&
      (HAL_UART_Transmit(&huart1, (const uint8_t *)buffer,
                         (uint16_t)used, 200U) != HAL_OK))
  {
    ++AdcUartErrorCount;
    UART_SendText("#SPECTRUM ABORT UART\r\n");
    return;
  }
  UART_SendText("#SPECTRUM END\r\n");
}

/* USER CODE END Application */

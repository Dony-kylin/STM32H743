/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad7606.h"
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AD7606_BUSY_TIMEOUT_TICKS  100U
#define AD7606_SAMPLE_RATE_HZ      25600U
/* 12.8 CSV frames/s at 25.6 kSPS; at most 100/s at the 200 kSPS limit. */
#define AD7606_UART_DECIMATION     2000U
#define AD7606_FRAME_QUEUE_DEPTH   8U
#define AD7606_UART_BUFFER_SIZE    128U
#define AD7606_CONFIG_RANGE        AD7606_RANGE_5V
#define AD7606_FULL_SCALE_100UV    ((AD7606_CONFIG_RANGE == AD7606_RANGE_10V) ? \
                                    100000L : 50000L)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t AdcSemaphoreHandle;         /* ADC 转换完成信号量 */
osMessageQueueId_t AdcFrameQueueHandle;
volatile AD7606_Frame AdcLatestFrame;
volatile uint32_t AdcFrameCount;
volatile uint32_t AdcTimeoutCount;
volatile uint32_t AdcReadErrorCount;
volatile uint32_t AdcTelemetryDropCount;
volatile uint32_t AdcUartErrorCount;
static volatile uint8_t UartStreamEnabled = 1U;
static AD7606_ScopeConfig ScopeStartupConfig;
static uint32_t ScopeStartupSampleRateHz = AD7606_SAMPLE_RATE_HZ;
static uint8_t ScopeStartupStreamEnabled = 1U;
static uint8_t ScopeStartupConfigValid;
static uint8_t ScopeConfigDirty;

/* Task_Uart: CubeMX 不会自动生成，放在 USER CODE 区防止被清除 */
osThreadId_t Task_UartHandle;
const osThreadAttr_t Task_Uart_attributes = {
  .name = "Task_Uart",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
static void AD7606_PrepareAcquisition(void);
static uint32_t AD7606_FormatVoltageFrame(char *buffer, uint32_t size,
                                          const AD7606_Frame *frame);
static void UART_PollScopeCommands(void);
static void UART_HandleScopeCommand(char *command);
static void UART_SendText(const char *text);
static void UART_SendScopeStatus(void);
static uint8_t UART_SaveScopeConfig(void);
static void UART_Acknowledge(const char *ok_message);
static void AD7606_ApplyStartupScopeConfig(void);
void StartTask_Uart(void *argument);
void AD7606_FrameReadyCallback(const int16_t *channels);

/* USER CODE END FunctionPrototypes */

void StartTask_DataProcess(void *argument);
void StartTask_Lcd(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  AD7606_ScopeInit((AD7606_CONFIG_RANGE == AD7606_RANGE_10V) ?
                   10000U : 5000U);
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
  AdcFrameQueueHandle = osMessageQueueNew(AD7606_FRAME_QUEUE_DEPTH,
                                           sizeof(AD7606_Frame), NULL);
  configASSERT(AdcFrameQueueHandle != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_DataProces */
  Task_DataProcesHandle = osThreadNew(StartTask_DataProcess, NULL, &Task_DataProces_attributes);

  /* creation of Task_Lcd */
  Task_LcdHandle = osThreadNew(StartTask_Lcd, NULL, &Task_Lcd_attributes);

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
  *         等待 BUSY 中断信号量，读取 FMC 数据并组装采样帧。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_DataProcess */
void StartTask_DataProcess(void *argument)
{
  /* USER CODE BEGIN StartTask_DataProcess */
  AD7606_Frame frame;
  uint32_t frame_count;

  (void)argument;
  AD7606_ApplyStartupScopeConfig();
  AD7606_PrepareAcquisition();

  /*
   * TIM1_CH1 generates the hardware-timed CONVST waveform. Samples are read
   * directly in the BUSY falling-edge interrupt, so LCD/UART scheduling
   * cannot stretch the configured conversion interval.
   */
  if (AD7606_StartContinuous(ScopeStartupSampleRateHz) ==
      AD7606_STATUS_OK)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  /* Keep the former task-driven acquisition as a safe startup fallback. */
  ++AdcReadErrorCount;
  for(;;)
  {
    while (osSemaphoreAcquire(AdcSemaphoreHandle, 0U) == osOK)
    {
    }

    frame.timestamp = HAL_GetTick();
    if (AD7606_StartConversion() != AD7606_STATUS_OK)
    {
      ++AdcReadErrorCount;
      AD7606_PrepareAcquisition();
      continue;
    }

    if (osSemaphoreAcquire(AdcSemaphoreHandle, AD7606_BUSY_TIMEOUT_TICKS) != osOK)
    {
      ++AdcTimeoutCount;
      AD7606_PrepareAcquisition();
      continue;
    }

    if (AD7606_ReadChannels(frame.channels) != AD7606_STATUS_OK)
    {
      ++AdcReadErrorCount;
      AD7606_PrepareAcquisition();
      continue;
    }

    AD7606_ScopePushFrame(frame.channels);

    taskENTER_CRITICAL();
    for (uint32_t i = 0U; i < AD7606_NUM_CHANNELS; ++i)
    {
      AdcLatestFrame.channels[i] = frame.channels[i];
    }
    AdcLatestFrame.timestamp = frame.timestamp;
    frame_count = ++AdcFrameCount;
    taskEXIT_CRITICAL();

    /* Only telemetry is decimated; every conversion is still read and counted. */
    if ((frame_count % AD7606_UART_DECIMATION) == 0U)
    {
      if (osMessageQueuePut(AdcFrameQueueHandle, &frame, 0U, 0U) != osOK)
      {
        ++AdcTelemetryDropCount;
      }
      taskYIELD();  /* 让出 CPU 给 UART 发送数据，避免饥饿 */
    }

    /* 下一轮立即重新触发，以获得 AD7606 的最高连续采样率。 */
  }
  /* USER CODE END StartTask_DataProcess */
}

void AD7606_FrameReadyCallback(const int16_t *channels)
{
  static uint32_t uart_decimation_count;
  AD7606_Frame frame;

  AD7606_ScopePushFrame(channels);
  ++AdcFrameCount;

  ++uart_decimation_count;
  if (uart_decimation_count >= AD7606_UART_DECIMATION)
  {
    uint32_t timestamp = HAL_GetTick();

    uart_decimation_count = 0U;
    for (uint32_t i = 0U; i < AD7606_NUM_CHANNELS; ++i)
    {
      frame.channels[i] = channels[i];
      AdcLatestFrame.channels[i] = channels[i];
    }
    frame.timestamp = timestamp;
    AdcLatestFrame.timestamp = timestamp;

    if ((UartStreamEnabled != 0U) &&
        (osMessageQueuePut(AdcFrameQueueHandle, &frame, 0U, 0U) != osOK))
    {
      ++AdcTelemetryDropCount;
    }
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
  AD7606_ScopeDisplayInit();

  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);  /* LED 闪烁 */
    AD7606_ScopeDisplayRefresh();
    osDelay(AD7606_ScopeGetRefreshMs());
  }
  /* USER CODE END StartTask_Lcd */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ======================== UART 遥测任务 ======================== */
void StartTask_Uart(void *argument)
{
  static const char header[] =
      "time_ms,ch1_V,ch2_V,ch3_V,ch4_V,ch5_V,ch6_V,ch7_V,ch8_V\r\n";
  AD7606_Frame frame;
  char tx_buffer[AD7606_UART_BUFFER_SIZE];

  (void)argument;
  /* 等待 UART 外设和终端就绪 */
  osDelay(100);

  if (HAL_UART_Transmit(&huart1, (const uint8_t *)header,
                        (uint16_t)(sizeof(header) - 1U), 100U) != HAL_OK)
  {
    ++AdcUartErrorCount;
  }
  UART_SendText("#SCOPE ready; type HELP for commands\r\n");

  for(;;)
  {
    if (osMessageQueueGet(AdcFrameQueueHandle, &frame, NULL,
                          10U) == osOK)
    {
      if (UartStreamEnabled != 0U)
      {
        uint32_t length = AD7606_FormatVoltageFrame(tx_buffer,
                                                     sizeof(tx_buffer), &frame);
        if ((length > 0U) &&
            (HAL_UART_Transmit(&huart1, (const uint8_t *)tx_buffer,
                               (uint16_t)length, 100U) != HAL_OK))
        {
          ++AdcUartErrorCount;
        }
      }
    }

    UART_PollScopeCommands();

    /* 每 2 秒输出一次诊断 */
    {
      static uint32_t last_diag_tick;
      uint32_t now = HAL_GetTick();
      if ((UartStreamEnabled != 0U) &&
          ((now - last_diag_tick) >= 2000U))
      {
        last_diag_tick = now;
        char diag[144];
        int diag_len = snprintf(diag, sizeof(diag),
            "#DIAG frames=%lu fs=%luHz ovr=%lu timeout=%lu rdErr=%lu drop=%lu\r\n",
            (unsigned long)AdcFrameCount,
            (unsigned long)AD7606_ScopeGetInputSampleRateHz(),
            (unsigned long)AD7606_GetOverrunCount(),
            (unsigned long)AdcTimeoutCount,
            (unsigned long)AdcReadErrorCount,
            (unsigned long)AdcTelemetryDropCount);
        HAL_UART_Transmit(&huart1, (const uint8_t *)diag,
                          (uint16_t)diag_len, 100U);
      }
    }
  }
}

/* ======================== AD7606 辅助函数 ======================== */

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
        "#CMD CH 1..8 | VDIV 50/100/200/500/1000/2000\r\n"
        "#CMD CENTER -5000..5000(mV) or CENTER AUTO\r\n"
        "#CMD RATE 5000..200000(sps)\r\n"
        "#CMD TIME AUTO or TIME 10..1000000(us/div)\r\n"
        "#CMD DEC 1/2/4/8/16/32/64 | REFRESH 50..2000(ms)\r\n"
        "#CMD AUTO (fit VDIV/CENTER/TIME; keep DEC)\r\n"
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
    UART_SendScopeStatus();
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
      UART_SendText("#ERR CH must be 1..8\r\n");
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
    if ((value >= 5000UL) && (value <= 200000UL) &&
        (AD7606_StartContinuous((uint32_t)value) == AD7606_STATUS_OK))
    {
      AD7606_ScopeConfig config;

      AD7606_ScopeGetConfig(&config);
      (void)AD7606_ScopeSetDecimation(config.decimation);
      UART_Acknowledge("#OK RATE\r\n");
    }
    else
    {
      UART_SendText("#ERR RATE 5000..200000 sps\r\n");
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
      UART_SendText("#ERR DEC 1/2/4/8/16/32/64\r\n");
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
      UART_SendText("#ERR REFRESH 50..2000 ms\r\n");
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
  char center[20];
  char timebase[20];
  char status[240];

  AD7606_ScopeGetConfig(&config);
  sample_rate_hz = AD7606_ScopeGetInputSampleRateHz();
  target_rate_hz = AD7606_GetConfiguredSampleRateHz();
  overrun_count = AD7606_GetOverrunCount();
  if (config.time_per_div_us == 0U)
  {
    (void)snprintf(timebase, sizeof(timebase), "AUTO");
  }
  else
  {
    (void)snprintf(timebase, sizeof(timebase), "%luus/div",
                   (unsigned long)config.time_per_div_us);
  }

  if (config.center_auto != 0U)
  {
    (void)snprintf(center, sizeof(center), "AUTO");
  }
  else
  {
    (void)snprintf(center, sizeof(center), "%+ldmV",
                   (long)config.center_mv);
  }

  (void)snprintf(status, sizeof(status),
      "#SCOPE %s CH=%u VDIV=%umV CENTER=%s TIME=%s DEC=%u RATE=%luHz FS=%luHz OVR=%lu REFRESH=%lums STREAM=%s CFG=%s\r\n",
      (config.running != 0U) ? "RUN" : "HOLD",
      (unsigned int)config.channel,
      (unsigned int)config.mv_per_div,
      center,
      timebase,
      (unsigned int)config.decimation,
      (unsigned long)target_rate_hz,
      (unsigned long)sample_rate_hz,
      (unsigned long)overrun_count,
      (unsigned long)config.refresh_ms,
      (UartStreamEnabled != 0U) ? "ON" : "OFF",
      (ScopeConfigDirty != 0U) ? "DIRTY" : "SAVED");
  UART_SendText(status);
}

static uint8_t UART_SaveScopeConfig(void)
{
  AD7606_ScopeConfig config;
  uint32_t sample_rate_hz = AD7606_GetConfiguredSampleRateHz();

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
  UART_SendScopeStatus();
}

static void AD7606_ApplyStartupScopeConfig(void)
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

static void AD7606_PrepareAcquisition(void)
{
  HAL_NVIC_DisableIRQ(AD7606_BUSY_EXTI_IRQn);
  AD7606_Init(AD7606_OS_NONE, AD7606_CONFIG_RANGE);

  __HAL_GPIO_EXTI_CLEAR_IT(AD7606_BUSY_Pin);
  HAL_NVIC_ClearPendingIRQ(AD7606_BUSY_EXTI_IRQn);
  while (osSemaphoreAcquire(AdcSemaphoreHandle, 0U) == osOK)
  {
  }

  HAL_NVIC_EnableIRQ(AD7606_BUSY_EXTI_IRQn);
}

static uint32_t AD7606_FormatVoltageFrame(char *buffer, uint32_t size,
                                          const AD7606_Frame *frame)
{
  uint32_t offset = 0U;
  int written = snprintf(buffer, size, "%lu", (unsigned long)frame->timestamp);

  if ((written < 0) || ((uint32_t)written >= size))
  {
    return 0U;
  }
  offset = (uint32_t)written;

  for (uint32_t i = 0U; i < AD7606_NUM_CHANNELS; ++i)
  {
    /* Scale to units of 0.0001 V without floating-point formatting. */
    int32_t scaled = (int32_t)(((int64_t)frame->channels[i] *
                                AD7606_FULL_SCALE_100UV) / 32768L);
    uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    const char *sign = (scaled < 0) ? "-" : "";

    written = snprintf(&buffer[offset], size - offset, ",%s%lu.%04lu",
                       sign, (unsigned long)(magnitude / 10000U),
                       (unsigned long)(magnitude % 10000U));
    if ((written < 0) || ((uint32_t)written >= (size - offset)))
    {
      return 0U;
    }
    offset += (uint32_t)written;
  }

  if ((size - offset) < 3U)
  {
    return 0U;
  }
  buffer[offset++] = '\r';
  buffer[offset++] = '\n';
  buffer[offset] = '\0';

  return offset;
}

/* USER CODE END Application */


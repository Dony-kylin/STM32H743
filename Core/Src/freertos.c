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
#include "usart.h"
#include "lcd_spi_154.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AD7606_BUSY_TIMEOUT_TICKS  100U
/* At 200 kSPS this emits about 1000 CSV frames/s without slowing acquisition. */
#define AD7606_UART_DECIMATION     200U
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

/* Task_Uart: CubeMX 不会自动生成，放在 USER CODE 区防止被清除 */
osThreadId_t Task_UartHandle;
const osThreadAttr_t Task_Uart_attributes = {
  .name = "Task_Uart",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for Task_DataProces */
osThreadId_t Task_DataProcesHandle;
const osThreadAttr_t Task_DataProces_attributes = {
  .name = "Task_DataProces",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Lcd */
osThreadId_t Task_LcdHandle;
const osThreadAttr_t Task_Lcd_attributes = {
  .name = "Task_Lcd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void AD7606_PrepareAcquisition(void);
static uint32_t AD7606_FormatVoltageFrame(char *buffer, uint32_t size,
                                          const AD7606_Frame *frame);
void StartTask_Uart(void *argument);

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

  AD7606_PrepareAcquisition();

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
  /* LCD 初始化已在 main.c 中完成 */

  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);  /* LED 闪烁 */
    osDelay(1000);
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

  /* 等待 UART 外设和终端就绪 */
  osDelay(100);

  if (HAL_UART_Transmit(&huart1, (const uint8_t *)header,
                        (uint16_t)(sizeof(header) - 1U), 100U) != HAL_OK)
  {
    ++AdcUartErrorCount;
  }

  for(;;)
  {
    if (osMessageQueueGet(AdcFrameQueueHandle, &frame, NULL,
                          osWaitForever) == osOK)
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

    /* 每 2 秒输出一次诊断 */
    {
      static uint32_t last_diag_tick;
      uint32_t now = HAL_GetTick();
      if ((now - last_diag_tick) >= 2000U)
      {
        last_diag_tick = now;
        char diag[96];
        int diag_len = snprintf(diag, sizeof(diag),
            "#DIAG frames=%lu timeout=%lu rdErr=%lu drop=%lu\r\n",
            (unsigned long)AdcFrameCount,
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


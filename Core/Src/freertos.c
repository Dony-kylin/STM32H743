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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t AdcSemaphoreHandle;         /* ADC 转换完成信号量 */
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_DataProces */
  Task_DataProcesHandle = osThreadNew(StartTask_DataProcess, NULL, &Task_DataProces_attributes);

  /* creation of Task_Lcd */
  Task_LcdHandle = osThreadNew(StartTask_Lcd, NULL, &Task_Lcd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  uint16_t adc_raw[AD7606_NUM_CHANNELS];
  AD7606_Frame frame;

  /* 初始化 AD7606：无过采样，输入量程为 +/-5 V */
  AD7606_Init(AD7606_OS_NONE, AD7606_RANGE_5V);

  /* 启动第一次转换 */
  AD7606_StartConversion();

  /* Infinite loop */
  for(;;)
  {
    /* 等待 BUSY 下降沿中断释放信号量，表示转换完成 */
    osSemaphoreAcquire(AdcSemaphoreHandle, osWaitForever);

    /* 通过 FMC 并行读取 8 通道数据 */
    AD7606_ReadChannels(adc_raw);

    /* 组装数据帧并记录采样时间戳 */
    for (int i = 0; i < AD7606_NUM_CHANNELS; i++)
    {
      frame.channels[i] = adc_raw[i];
    }
    frame.timestamp = HAL_GetTick();

    /* ==================================================== */
    /* TODO: 在此添加数据处理逻辑                            */
    /* 例如：存入环形缓冲区，或通知 LCD 任务显示波形         */
    /* ==================================================== */
    (void)frame;  /* 暂未使用；实现数据处理后可删除此语句 */

    /* 启动下一次转换，继续采集 */
    AD7606_StartConversion();

    /* 阻塞 10 个系统节拍，确保低优先级任务能够运行 */
    osDelay(10);
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
  /* Infinite loop */
  for(;;)
  {
    printf("LCD Task Running...\n");
    osDelay(1000);
  }
  /* USER CODE END StartTask_Lcd */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


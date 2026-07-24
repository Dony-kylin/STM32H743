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
osSemaphoreId_t AdcSemaphoreHandle;         /* ADC转换完成信号量 */
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
  /* 创建二进制信号量: 初始为0 (不可用), 最大计数1 */
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
  *         等待BUSY中断信号量 → 读取FMC数据 → 存入缓冲区
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_DataProcess */
void StartTask_DataProcess(void *argument)
{
  /* USER CODE BEGIN StartTask_DataProcess */
  uint16_t adc_raw[AD7606_NUM_CHANNELS];
  AD7606_Frame frame;

  /* 初始化AD7606: 无过采样, ±5V量程 */
  AD7606_Init(AD7606_OS_NONE, AD7606_RANGE_5V);

  /* 启动第一次转换 */
  AD7606_StartConversion();

  /* Infinite loop */
  for(;;)
  {
    /* 等待BUSY中断释放信号量 (转换完成) */
    osSemaphoreAcquire(AdcSemaphoreHandle, osWaitForever);

    /* 通过FMC并行读取8通道数据 */
    AD7606_ReadChannels(adc_raw);

    /* 组装数据帧 (记录时间戳) */
    for (int i = 0; i < AD7606_NUM_CHANNELS; i++)
    {
      frame.channels[i] = adc_raw[i];
    }
    frame.timestamp = HAL_GetTick();

    /* ==================================================== */
    /* TODO: 在此添加数据处理逻辑                            */
    /* 例如: 存入环形缓冲区 / 通知LCD任务显示波形            */
    /* ==================================================== */
    (void)frame;  /* 消除未使用警告，实现处理逻辑后可删除 */

    /* 启动下一次转换 (连续采集) */
    AD7606_StartConversion();

    /* 让低优先级任务有机会运行 */
    osDelay(0);
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
    osDelay(1);
  }
  /* USER CODE END StartTask_Lcd */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


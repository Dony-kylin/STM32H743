/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ad7606.c
  * @brief          : AD7606 并行ADC驱动 (FMC接口 + FreeRTOS)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ad7606.h"
#include "cmsis_os.h"      /* CMSIS RTOS v2 API */

/* External variables --------------------------------------------------------*/
/* freertos.c 中定义的信号量句柄 */
extern osSemaphoreId_t AdcSemaphoreHandle;

/* ======================== HAL 中断回调 ======================== */

/**
  * @brief  GPIO EXTI 中断回调 (由 HAL_GPIO_EXTI_IRQHandler 调用)
  * @param  GPIO_Pin  触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == AD7606_BUSY_Pin)
    {
        /* BUSY下降沿 → 转换完成, 通知采集任务 */
        AD7606_ConvCompleteCallback();
    }
}

/* ======================== 初始化 ======================== */

/**
  * @brief  初始化AD7606：复位芯片，配置过采样和量程
  * @param  os   过采样模式 (AD7606_OS_NONE ~ AD7606_OS_64X)
  * @param  range 量程 (±5V 或 ±10V)
  * @retval None
  */
void AD7606_Init(AD7606_OSMode os, AD7606_Range range)
{
    /* ---- 复位AD7606 (RST高电平脉冲) ---- */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_RESET);
    osDelay(2);                                  /* tRST低 > 50ns */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_SET);
    osDelay(2);                                  /* 等待复位完成 */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_RESET);
    osDelay(10);                                 /* 稳定等待 */

    /* ---- 配置过采样 OS[2:0] ---- */
    HAL_GPIO_WritePin(AD7606_OS0_GPIO_Port, AD7606_OS0_Pin,
        (os & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS1_GPIO_Port, AD7606_OS1_Pin,
        (os & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS2_GPIO_Port, AD7606_OS2_Pin,
        (os & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ---- 配置量程 RANGE: 0=±5V, 1=±10V ---- */
    HAL_GPIO_WritePin(AD7606_RANGE_GPIO_Port, AD7606_RANGE_Pin,
        (range == AD7606_RANGE_10V) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ---- 正常模式 FD=0 (非滤波/省电模式) ---- */
    HAL_GPIO_WritePin(AD7606_FD_GPIO_Port, AD7606_FD_Pin, GPIO_PIN_RESET);
}

/* ======================== 启动转换 ======================== */

/**
  * @brief  发送CONVST脉冲启动一次转换 (所有通道同时)
  * @note   CONVST: 高→低→高, 低电平保持>25ns
  * @retval None
  */
void AD7606_StartConversion(void)
{
    /* CONVST 从高拉低, 再拉高, 产生下降沿触发转换 */
    HAL_GPIO_WritePin(AD7606_CACB_GPIO_Port, AD7606_CACB_Pin, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP();                  /* >25ns @480MHz */
    HAL_GPIO_WritePin(AD7606_CACB_GPIO_Port, AD7606_CACB_Pin, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP();                  /* 保持低电平 */
    HAL_GPIO_WritePin(AD7606_CACB_GPIO_Port, AD7606_CACB_Pin, GPIO_PIN_SET);
}

/* ======================== 读取数据 ======================== */

/**
  * @brief  通过FMC连续读取8个通道的数据
  * @note   每个 AD7606_Read() 触发一次 FMC 读周期:
  *         - NE1(CS) 自动拉低
  *         - NOE(RD) 产生读脉冲
  *         - 16位数据从 DB[15:0] 读取
  *         AD7606自动按 V1→V2→...→V8 顺序输出
  * @param  buf  输出缓冲区 (至少8个uint16_t)
  * @retval None
  */
void AD7606_ReadChannels(uint16_t *buf)
{
    for (int i = 0; i < AD7606_NUM_CHANNELS; i++)
    {
        buf[i] = AD7606_Read();
    }
}

/* ======================== 中断回调 ======================== */

/**
  * @brief  BUSY下降沿中断回调
  * @note   在 stm32h7xx_it.c 的 EXTI0_IRQHandler 中调用
  *         释放信号量, 通知 StartTask_DataProcess 读取数据
  * @retval None
  */
void AD7606_ConvCompleteCallback(void)
{
    /* 释放二进制信号量 → 通知采集任务 */
    osSemaphoreRelease(AdcSemaphoreHandle);
}

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

static const volatile uint16_t *const ad7606_data_reg =
    (const volatile uint16_t *)AD7606_FMC_BASE;
static volatile uint8_t ad7606_conversion_pending;

static void AD7606_SetConvst(GPIO_PinState state)
{
    uint32_t bsrr = (state == GPIO_PIN_SET) ?
        (uint32_t)AD7606_CONVST_AB_Pin :
        ((uint32_t)AD7606_CONVST_AB_Pin << 16U);

    AD7606_CONVST_AB_GPIO_Port->BSRR = bsrr;
}

static void AD7606_ConvstPulseDelay(void)
{
    /* Loop overhead makes this safely longer than the 25 ns minimum pulse. */
    for (uint32_t i = 0U; i < 32U; ++i)
    {
        __NOP();
    }
}

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
    ad7606_conversion_pending = 0U;

    /* 板上 CONVST_A/B 并联到 PA8，空闲电平为低。 */
    AD7606_SetConvst(GPIO_PIN_RESET);

    /* 配置过采样 OS[2:0]。 */
    HAL_GPIO_WritePin(AD7606_OS0_GPIO_Port, AD7606_OS0_Pin,
        (os & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS1_GPIO_Port, AD7606_OS1_Pin,
        (os & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS2_GPIO_Port, AD7606_OS2_Pin,
        (os & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* RANGE: 低电平为 +/-5 V，高电平为 +/-10 V。 */
    HAL_GPIO_WritePin(AD7606_RANGE_GPIO_Port, AD7606_RANGE_Pin,
        (range == AD7606_RANGE_10V) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ---- 复位AD7606 (RST高电平脉冲) ---- */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_RESET);
    osDelay(2);                                  /* 复位前保持低电平 */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_SET);
    osDelay(2);                                  /* RESET 高电平脉宽远大于 50 ns */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_RESET);
    osDelay(10);                                 /* 等待复位完成并稳定 */
}

/* ======================== 启动转换 ======================== */

/**
  * @brief  通过 PA8 上并联的 CONVST_A/B 启动 8 通道转换
  * @retval AD7606_STATUS_OK 或 AD7606_STATUS_BUSY
  */
AD7606_Status AD7606_StartConversion(void)
{
    if (AD7606_IsBusy() != 0U)
    {
        return AD7606_STATUS_BUSY;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(AD7606_BUSY_Pin);
    HAL_NVIC_ClearPendingIRQ(AD7606_BUSY_EXTI_IRQn);
    ad7606_conversion_pending = 1U;

    AD7606_SetConvst(GPIO_PIN_RESET);
    AD7606_ConvstPulseDelay();
    AD7606_SetConvst(GPIO_PIN_SET);
    AD7606_ConvstPulseDelay();
    AD7606_SetConvst(GPIO_PIN_RESET);

    return AD7606_STATUS_OK;
}

/* ======================== 读取数据 ======================== */

/**
  * @brief  通过FMC连续读取8个通道的数据
  * @note   每次 volatile 加载触发一个 FMC 读周期:
  *         - NE1(CS) 自动拉低
  *         - NOE(RD) 产生读脉冲
  *         - 16位数据从 DB[15:0] 读取
  *         AD7606自动按 V1→V2→...→V8 顺序输出
  * @param  buf  输出缓冲区 (至少8个int16_t)
  * @retval AD7606_STATUS_OK、AD7606_STATUS_BUSY 或
  *         AD7606_STATUS_INVALID_ARGUMENT
  */
AD7606_Status AD7606_ReadChannels(int16_t *buf)
{
    if (buf == NULL)
    {
        return AD7606_STATUS_INVALID_ARGUMENT;
    }

    if (AD7606_IsBusy() != 0U)
    {
        return AD7606_STATUS_BUSY;
    }

    /* 强顺序 MPU 属性和 volatile 访问确保每次加载产生一个独立 RD 脉冲。 */
    __DSB();
    for (uint32_t i = 0U; i < AD7606_NUM_CHANNELS; ++i)
    {
        buf[i] = (int16_t)(*ad7606_data_reg);
    }
    __DSB();

    return AD7606_STATUS_OK;
}

uint8_t AD7606_IsBusy(void)
{
    return (HAL_GPIO_ReadPin(AD7606_BUSY_GPIO_Port, AD7606_BUSY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
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
    if ((ad7606_conversion_pending != 0U) &&
        (AD7606_IsBusy() == 0U) &&
        (AdcSemaphoreHandle != NULL))
    {
        if (osSemaphoreRelease(AdcSemaphoreHandle) == osOK)
        {
            ad7606_conversion_pending = 0U;
        }
    }
}

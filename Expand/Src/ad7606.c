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
static TIM_HandleTypeDef ad7606_convst_timer;
static int16_t ad7606_continuous_channels[AD7606_NUM_CHANNELS];
static volatile uint8_t ad7606_conversion_pending;
static volatile uint8_t ad7606_continuous_mode;
static volatile uint32_t ad7606_configured_sample_rate_hz;
static volatile uint32_t ad7606_overrun_count;

#define AD7606_MAX_SAMPLE_RATE_HZ        200000U
#define AD7606_CONVST_HIGH_TIME_NS       250U

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

static void AD7606_ConfigureConvstAsGpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = AD7606_CONVST_AB_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(AD7606_CONVST_AB_GPIO_Port, &gpio);
    AD7606_SetConvst(GPIO_PIN_RESET);
}

static uint32_t AD7606_GetTim1ClockHz(void)
{
    RCC_ClkInitTypeDef clock_config;
    uint32_t flash_latency;
    uint32_t timer_clock;

    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
    timer_clock = HAL_RCC_GetPCLK2Freq();
    if (clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clock *= 2U;
    }
    return timer_clock;
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
    AD7606_StopContinuous();
    ad7606_conversion_pending = 0U;
    ad7606_overrun_count = 0U;

    /* 板上 CONVST_A/B 并联到 PA8，空闲电平为低。 */
    AD7606_ConfigureConvstAsGpio();

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
    if (ad7606_continuous_mode != 0U)
    {
        return AD7606_STATUS_BUSY;
    }

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

AD7606_Status AD7606_StartContinuous(uint32_t sample_rate_hz)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef output_compare = {0};
    uint32_t timer_clock;
    uint32_t period_ticks;
    uint32_t pulse_ticks;

    if ((sample_rate_hz == 0U) ||
        (sample_rate_hz > AD7606_MAX_SAMPLE_RATE_HZ))
    {
        return AD7606_STATUS_INVALID_ARGUMENT;
    }

    timer_clock = AD7606_GetTim1ClockHz();
    period_ticks = timer_clock / sample_rate_hz;
    if ((period_ticks < 4U) || (period_ticks > 65536U))
    {
        return AD7606_STATUS_INVALID_ARGUMENT;
    }

    AD7606_StopContinuous();
    {
        uint32_t wait_start = HAL_GetTick();
        while (AD7606_IsBusy() != 0U)
        {
            if ((HAL_GetTick() - wait_start) >= 2U)
            {
                return AD7606_STATUS_BUSY;
            }
        }
    }

    pulse_ticks = (uint32_t)
        (((uint64_t)timer_clock * AD7606_CONVST_HIGH_TIME_NS) /
         1000000000ULL);
    if (pulse_ticks < 2U)
    {
        pulse_ticks = 2U;
    }
    if (pulse_ticks >= period_ticks)
    {
        pulse_ticks = period_ticks / 2U;
    }

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();

    ad7606_convst_timer.Instance = TIM1;
    ad7606_convst_timer.Init.Prescaler = 0U;
    ad7606_convst_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    ad7606_convst_timer.Init.Period = period_ticks - 1U;
    ad7606_convst_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    ad7606_convst_timer.Init.RepetitionCounter = 0U;
    ad7606_convst_timer.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&ad7606_convst_timer) != HAL_OK)
    {
        AD7606_ConfigureConvstAsGpio();
        return AD7606_STATUS_BUSY;
    }

    output_compare.OCMode = TIM_OCMODE_PWM1;
    output_compare.Pulse = pulse_ticks;
    output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
    output_compare.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    output_compare.OCFastMode = TIM_OCFAST_DISABLE;
    output_compare.OCIdleState = TIM_OCIDLESTATE_RESET;
    output_compare.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&ad7606_convst_timer, &output_compare,
                                  TIM_CHANNEL_1) != HAL_OK)
    {
        AD7606_ConfigureConvstAsGpio();
        return AD7606_STATUS_BUSY;
    }

    gpio.Pin = AD7606_CONVST_AB_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(AD7606_CONVST_AB_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(AD7606_BUSY_Pin);
    HAL_NVIC_ClearPendingIRQ(AD7606_BUSY_EXTI_IRQn);
    ad7606_conversion_pending = 0U;
    ad7606_continuous_mode = 1U;
    ad7606_overrun_count = 0U;
    __HAL_TIM_SET_COUNTER(&ad7606_convst_timer, 0U);

    if (HAL_TIM_PWM_Start(&ad7606_convst_timer, TIM_CHANNEL_1) != HAL_OK)
    {
        ad7606_continuous_mode = 0U;
        AD7606_ConfigureConvstAsGpio();
        return AD7606_STATUS_BUSY;
    }

    ad7606_configured_sample_rate_hz = timer_clock / period_ticks;
    return AD7606_STATUS_OK;
}

void AD7606_StopContinuous(void)
{
    if (ad7606_continuous_mode != 0U)
    {
        (void)HAL_TIM_PWM_Stop(&ad7606_convst_timer, TIM_CHANNEL_1);
        ad7606_continuous_mode = 0U;
    }
    ad7606_configured_sample_rate_hz = 0U;
    AD7606_ConfigureConvstAsGpio();
}

uint32_t AD7606_GetConfiguredSampleRateHz(void)
{
    return ad7606_configured_sample_rate_hz;
}

uint32_t AD7606_GetOverrunCount(void)
{
    return ad7606_overrun_count;
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
    if (ad7606_continuous_mode != 0U)
    {
        /*
         * Keep the 200 kSPS ISR path short. The previous conversion result
         * remains readable while the next conversion is running, so do not
         * reject a delayed BUSY interrupt merely because BUSY is high again.
         */
        __DSB();
        for (uint32_t i = 0U; i < AD7606_NUM_CHANNELS; ++i)
        {
            ad7606_continuous_channels[i] =
                (int16_t)(*ad7606_data_reg);
        }
        __DSB();
        AD7606_FrameReadyCallback(ad7606_continuous_channels);

        if (__HAL_GPIO_EXTI_GET_IT(AD7606_BUSY_Pin) != 0U)
        {
            ++ad7606_overrun_count;
        }
        return;
    }

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

__weak void AD7606_FrameReadyCallback(const int16_t *channels)
{
    (void)channels;
}

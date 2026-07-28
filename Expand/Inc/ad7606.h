/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ad7606.h
  * @brief          : AD7606 并行ADC驱动头文件
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __AD7606_H__
#define __AD7606_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ======================== 宏定义 ======================== */

/** @brief FMC Bank1 NE1 基地址 (对应AD7606片选CS) */
#define AD7606_FMC_BASE          ((uint32_t)0x60000000)

/** @brief AD7606 通道数 */
#define AD7606_NUM_CHANNELS      8

/* ======================== 枚举类型 ======================== */

/** @brief 过采样模式 */
typedef enum {
    AD7606_OS_NONE  = 0,  /*!< 无过采样 */
    AD7606_OS_2X    = 1,  /*!< 2倍过采样 */
    AD7606_OS_4X    = 2,  /*!< 4倍过采样 */
    AD7606_OS_8X    = 3,  /*!< 8倍过采样 */
    AD7606_OS_16X   = 4,  /*!< 16倍过采样 */
    AD7606_OS_32X   = 5,  /*!< 32倍过采样 */
    AD7606_OS_64X   = 6,  /*!< 64倍过采样 */
} AD7606_OSMode;

/** @brief 量程选择 */
typedef enum {
    AD7606_RANGE_5V  = 0,  /*!< ±5V */
    AD7606_RANGE_10V = 1,  /*!< ±10V */
} AD7606_Range;

/** @brief AD7606 驱动状态 */
typedef enum {
    AD7606_STATUS_OK = 0,
    AD7606_STATUS_INVALID_ARGUMENT,
    AD7606_STATUS_BUSY,
} AD7606_Status;

/** @brief 一帧采集数据 (8通道) */
typedef struct {
    int16_t channels[AD7606_NUM_CHANNELS];   /*!< CH1~CH8 二补码原始值 */
    uint32_t timestamp;                        /*!< 采集时间戳 (uwTick) */
} AD7606_Frame;

/* ======================== 函数声明 ======================== */

/** @brief 初始化AD7606 (复位、配置过采样、量程) */
void AD7606_Init(AD7606_OSMode os, AD7606_Range range);

/** @brief 检查 AD7606 是否正在转换 */
uint8_t AD7606_IsBusy(void);

/** @brief 触发 PA8 上并联的 CONVST_A/B，启动 8 通道转换 */
AD7606_Status AD7606_StartConversion(void);

/** @brief 通过FMC并行读取8通道数据 */
AD7606_Status AD7606_ReadChannels(int16_t *buf);

/** @brief BUSY下降沿中断回调 (释放信号量) */
void AD7606_ConvCompleteCallback(void);

#ifdef __cplusplus
}
#endif
#endif /* __AD7606_H__ */

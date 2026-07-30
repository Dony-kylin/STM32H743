/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD7606_RST_Pin GPIO_PIN_0
#define AD7606_RST_GPIO_Port GPIOC
#define AD7606_OS0_Pin GPIO_PIN_1
#define AD7606_OS0_GPIO_Port GPIOC
#define AD7606_OS1_Pin GPIO_PIN_2
#define AD7606_OS1_GPIO_Port GPIOC
#define AD7606_OS2_Pin GPIO_PIN_3
#define AD7606_OS2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOA
#define AD7606_SPI_SCK_Pin GPIO_PIN_5
#define AD7606_SPI_SCK_GPIO_Port GPIOA
#define AD7606_SPI_MISO_Pin GPIO_PIN_6
#define AD7606_SPI_MISO_GPIO_Port GPIOA
#define AD7606_RANGE_Pin GPIO_PIN_4
#define AD7606_RANGE_GPIO_Port GPIOC
#define AD7606_FRSTDATA_Pin GPIO_PIN_5
#define AD7606_FRSTDATA_GPIO_Port GPIOC
#define AD7606_BUSY_Pin GPIO_PIN_0
#define AD7606_BUSY_GPIO_Port GPIOB
#define AD7606_BUSY_EXTI_IRQn EXTI0_IRQn
#define USER_LED_Pin GPIO_PIN_7
#define USER_LED_GPIO_Port GPIOG
#define AD7606_CONVST_AB_Pin GPIO_PIN_8
#define AD7606_CONVST_AB_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define AD7606_NOE_RD_Pin GPIO_PIN_4
#define AD7606_NOE_RD_GPIO_Port GPIOD
#define AD7606_NE1_CS_Pin GPIO_PIN_7
#define AD7606_NE1_CS_GPIO_Port GPIOD
#define AD9220_D11_Pin GPIO_PIN_0
#define AD9220_D11_GPIO_Port GPIOD
#define AD9220_D0_Pin GPIO_PIN_1
#define AD9220_D0_GPIO_Port GPIOD
#define AD9220_D5_Pin GPIO_PIN_8
#define AD9220_D5_GPIO_Port GPIOD
#define AD9220_D12_OTR_Pin GPIO_PIN_14
#define AD9220_D12_OTR_GPIO_Port GPIOD
#define AD9220_CLK_Pin GPIO_PIN_15
#define AD9220_CLK_GPIO_Port GPIOD
#define AD9220_D10_Pin GPIO_PIN_7
#define AD9220_D10_GPIO_Port GPIOE
#define AD9220_D1_Pin GPIO_PIN_8
#define AD9220_D1_GPIO_Port GPIOE
#define AD9220_D9_Pin GPIO_PIN_9
#define AD9220_D9_GPIO_Port GPIOE
#define AD9220_D2_Pin GPIO_PIN_10
#define AD9220_D2_GPIO_Port GPIOE
#define AD9220_D8_Pin GPIO_PIN_11
#define AD9220_D8_GPIO_Port GPIOE
#define AD9220_D3_Pin GPIO_PIN_12
#define AD9220_D3_GPIO_Port GPIOE
#define AD9220_D7_Pin GPIO_PIN_13
#define AD9220_D7_GPIO_Port GPIOE
#define AD9220_D4_Pin GPIO_PIN_14
#define AD9220_D4_GPIO_Port GPIOE
#define AD9220_D6_Pin GPIO_PIN_15
#define AD9220_D6_GPIO_Port GPIOE
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

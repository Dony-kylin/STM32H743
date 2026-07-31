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
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
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
#define AD9220_D5_Pin GPIO_PIN_8
#define AD9220_D5_GPIO_Port GPIOD
#define AD9220_D12_OTR_Pin GPIO_PIN_14
#define AD9220_D12_OTR_GPIO_Port GPIOD
#define AD9220_CLK_Pin GPIO_PIN_15
#define AD9220_CLK_GPIO_Port GPIOD
#define RUN_LED_Pin GPIO_PIN_7
#define RUN_LED_GPIO_Port GPIOG
#define LCD_NSS_Pin GPIO_PIN_8
#define LCD_NSS_GPIO_Port GPIOG
#define AD9220_D11_Pin GPIO_PIN_0
#define AD9220_D11_GPIO_Port GPIOD
#define AD9220_D0_Pin GPIO_PIN_1
#define AD9220_D0_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_12
#define LCD_BL_GPIO_Port GPIOG
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOG
#define LCD_MOSI_Pin GPIO_PIN_14
#define LCD_MOSI_GPIO_Port GPIOG
#define LCD_DC_Pin GPIO_PIN_15
#define LCD_DC_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

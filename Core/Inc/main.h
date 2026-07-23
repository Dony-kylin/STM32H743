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
#define LCD_A6_Pin GPIO_PIN_12
#define LCD_A6_GPIO_Port GPIOF
#define LCD_AD7606_D4_Pin GPIO_PIN_7
#define LCD_AD7606_D4_GPIO_Port GPIOE
#define LCD_AD7606_D5_Pin GPIO_PIN_8
#define LCD_AD7606_D5_GPIO_Port GPIOE
#define LCD_AD7606_D6_Pin GPIO_PIN_9
#define LCD_AD7606_D6_GPIO_Port GPIOE
#define LCD_AD7606_D7_Pin GPIO_PIN_10
#define LCD_AD7606_D7_GPIO_Port GPIOE
#define LCD_AD7606_D8_Pin GPIO_PIN_11
#define LCD_AD7606_D8_GPIO_Port GPIOE
#define LCD_AD7606_D9_Pin GPIO_PIN_12
#define LCD_AD7606_D9_GPIO_Port GPIOE
#define LCD_AD7606_D10_Pin GPIO_PIN_13
#define LCD_AD7606_D10_GPIO_Port GPIOE
#define LCD_AD7606_D11_Pin GPIO_PIN_14
#define LCD_AD7606_D11_GPIO_Port GPIOE
#define LCD_AD7606_D12_Pin GPIO_PIN_15
#define LCD_AD7606_D12_GPIO_Port GPIOE
#define LCD_AD7606_D13_Pin GPIO_PIN_8
#define LCD_AD7606_D13_GPIO_Port GPIOD
#define LCD_AD7606_D14_Pin GPIO_PIN_9
#define LCD_AD7606_D14_GPIO_Port GPIOD
#define LCD_AD7606_D15_Pin GPIO_PIN_10
#define LCD_AD7606_D15_GPIO_Port GPIOD
#define LCD_AD7606_D0_Pin GPIO_PIN_14
#define LCD_AD7606_D0_GPIO_Port GPIOD
#define LCD_AD7606_D1_Pin GPIO_PIN_15
#define LCD_AD7606_D1_GPIO_Port GPIOD
#define LCD_AD7606_PG7_Pin GPIO_PIN_7
#define LCD_AD7606_PG7_GPIO_Port GPIOG
#define LCD_AD7606_NE1_Pin GPIO_PIN_7
#define LCD_AD7606_NE1_GPIO_Port GPIOC
#define LCD_AD7606_D2_Pin GPIO_PIN_0
#define LCD_AD7606_D2_GPIO_Port GPIOD
#define LCD_AD7606_D3_Pin GPIO_PIN_1
#define LCD_AD7606_D3_GPIO_Port GPIOD
#define LCD_AD7606_NOE_Pin GPIO_PIN_4
#define LCD_AD7606_NOE_GPIO_Port GPIOD
#define LCD_AD7606_NWE_Pin GPIO_PIN_5
#define LCD_AD7606_NWE_GPIO_Port GPIOD
#define LCD_AD7606_NE4_Pin GPIO_PIN_12
#define LCD_AD7606_NE4_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

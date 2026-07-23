/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : FMC.c
  * Description        : This file provides code for the configuration
  *                      of the FMC peripheral.
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
#include "fmc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* FMC initialization function */
void MX_FMC_Init(void)
{
  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};
  FMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK4;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 3;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 20;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_D;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 3;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 20;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_D;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FMC_NORSRAM_DEVICE;
  hsram2.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram2.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FMC_WRITE_OPERATION_DISABLE;
  hsram2.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram2.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram2.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram2.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 4;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 5;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

static uint32_t FMC_Initialized = 0;

static void HAL_FMC_MspInit(void){
  /* USER CODE BEGIN FMC_MspInit 0 */

  /* USER CODE END FMC_MspInit 0 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (FMC_Initialized) {
    return;
  }
  FMC_Initialized = 1;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /* Peripheral clock enable */
  __HAL_RCC_FMC_CLK_ENABLE();

  /** FMC GPIO Configuration
  PF0   ------> FMC_A0
  PF12   ------> FMC_A6
  PE7   ------> FMC_D4
  PE8   ------> FMC_D5
  PE9   ------> FMC_D6
  PE10   ------> FMC_D7
  PE11   ------> FMC_D8
  PE12   ------> FMC_D9
  PE13   ------> FMC_D10
  PE14   ------> FMC_D11
  PE15   ------> FMC_D12
  PD8   ------> FMC_D13
  PD9   ------> FMC_D14
  PD10   ------> FMC_D15
  PD14   ------> FMC_D0
  PD15   ------> FMC_D1
  PC7   ------> FMC_NE1
  PD0   ------> FMC_D2
  PD1   ------> FMC_D3
  PD4   ------> FMC_NOE
  PD5   ------> FMC_NWE
  PG12   ------> FMC_NE4
  */
  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LCD_A6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = LCD_AD7606_D4_Pin|LCD_AD7606_D5_Pin|LCD_AD7606_D6_Pin|LCD_AD7606_D7_Pin
                          |LCD_AD7606_D8_Pin|LCD_AD7606_D9_Pin|LCD_AD7606_D10_Pin|LCD_AD7606_D11_Pin
                          |LCD_AD7606_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = LCD_AD7606_D13_Pin|LCD_AD7606_D14_Pin|LCD_AD7606_D15_Pin|LCD_AD7606_D0_Pin
                          |LCD_AD7606_D1_Pin|LCD_AD7606_D2_Pin|LCD_AD7606_D3_Pin|LCD_AD7606_NOE_Pin
                          |LCD_AD7606_NWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = LCD_AD7606_NE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FMC;

  HAL_GPIO_Init(LCD_AD7606_NE1_GPIO_Port, &GPIO_InitStruct);

  /* GPIO_InitStruct */
  GPIO_InitStruct.Pin = LCD_AD7606_NE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

  HAL_GPIO_Init(LCD_AD7606_NE4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN FMC_MspInit 1 */

  /* USER CODE END FMC_MspInit 1 */
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef* sramHandle){
  /* USER CODE BEGIN SRAM_MspInit 0 */

  /* USER CODE END SRAM_MspInit 0 */
  HAL_FMC_MspInit();
  /* USER CODE BEGIN SRAM_MspInit 1 */

  /* USER CODE END SRAM_MspInit 1 */
}

static uint32_t FMC_DeInitialized = 0;

static void HAL_FMC_MspDeInit(void){
  /* USER CODE BEGIN FMC_MspDeInit 0 */

  /* USER CODE END FMC_MspDeInit 0 */
  if (FMC_DeInitialized) {
    return;
  }
  FMC_DeInitialized = 1;
  /* Peripheral clock enable */
  __HAL_RCC_FMC_CLK_DISABLE();

  /** FMC GPIO Configuration
  PF0   ------> FMC_A0
  PF12   ------> FMC_A6
  PE7   ------> FMC_D4
  PE8   ------> FMC_D5
  PE9   ------> FMC_D6
  PE10   ------> FMC_D7
  PE11   ------> FMC_D8
  PE12   ------> FMC_D9
  PE13   ------> FMC_D10
  PE14   ------> FMC_D11
  PE15   ------> FMC_D12
  PD8   ------> FMC_D13
  PD9   ------> FMC_D14
  PD10   ------> FMC_D15
  PD14   ------> FMC_D0
  PD15   ------> FMC_D1
  PC7   ------> FMC_NE1
  PD0   ------> FMC_D2
  PD1   ------> FMC_D3
  PD4   ------> FMC_NOE
  PD5   ------> FMC_NWE
  PG12   ------> FMC_NE4
  */

  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0|LCD_A6_Pin);

  HAL_GPIO_DeInit(GPIOE, LCD_AD7606_D4_Pin|LCD_AD7606_D5_Pin|LCD_AD7606_D6_Pin|LCD_AD7606_D7_Pin
                          |LCD_AD7606_D8_Pin|LCD_AD7606_D9_Pin|LCD_AD7606_D10_Pin|LCD_AD7606_D11_Pin
                          |LCD_AD7606_D12_Pin);

  HAL_GPIO_DeInit(GPIOD, LCD_AD7606_D13_Pin|LCD_AD7606_D14_Pin|LCD_AD7606_D15_Pin|LCD_AD7606_D0_Pin
                          |LCD_AD7606_D1_Pin|LCD_AD7606_D2_Pin|LCD_AD7606_D3_Pin|LCD_AD7606_NOE_Pin
                          |LCD_AD7606_NWE_Pin);

  HAL_GPIO_DeInit(LCD_AD7606_NE1_GPIO_Port, LCD_AD7606_NE1_Pin);

  HAL_GPIO_DeInit(LCD_AD7606_NE4_GPIO_Port, LCD_AD7606_NE4_Pin);

  /* USER CODE BEGIN FMC_MspDeInit 1 */

  /* USER CODE END FMC_MspDeInit 1 */
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* sramHandle){
  /* USER CODE BEGIN SRAM_MspDeInit 0 */

  /* USER CODE END SRAM_MspDeInit 0 */
  HAL_FMC_MspDeInit();
  /* USER CODE BEGIN SRAM_MspDeInit 1 */

  /* USER CODE END SRAM_MspDeInit 1 */
}
/**
  * @}
  */

/**
  * @}
  */

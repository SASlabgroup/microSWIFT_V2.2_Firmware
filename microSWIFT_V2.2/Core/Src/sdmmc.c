/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    sdmmc.c
 * @brief   This file provides code for the configuration
 *          of the SDMMC instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "sdmmc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SD_HandleTypeDef hsd2;

/* SDMMC2 init function */

void MX_SDMMC2_SD_Init ( void )
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  if ( HAL_SD_Init (&hsd2) != HAL_OK )
  {
    Error_Handler ();
  }
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

void HAL_SD_MspInit ( SD_HandleTypeDef *sdHandle )
{

  GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit =
    { 0 };
  if ( sdHandle->Instance == SDMMC2 )
  {
    /* USER CODE BEGIN SDMMC2_MspInit 0 */

    /* USER CODE END SDMMC2_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_CLK48;
    PeriphClkInit.IclkClockSelection = RCC_CLK48CLKSOURCE_MSIK;
    PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_CLK48;
    if ( HAL_RCCEx_PeriphCLKConfig (&PeriphClkInit) != HAL_OK )
    {
      Error_Handler ();
    }

    /* SDMMC2 clock enable */
    __HAL_RCC_SDMMC2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC2 GPIO Configuration
     PB14     ------> SDMMC2_D0
     PD6     ------> SDMMC2_CK
     PD7     ------> SDMMC2_CMD
     */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC2;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
    HAL_GPIO_Init (GPIOD, &GPIO_InitStruct);

    /* SDMMC2 interrupt Init */
    HAL_NVIC_SetPriority (SDMMC2_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ (SDMMC2_IRQn);
    /* USER CODE BEGIN SDMMC2_MspInit 1 */

    /* USER CODE END SDMMC2_MspInit 1 */
  }
}

void HAL_SD_MspDeInit ( SD_HandleTypeDef *sdHandle )
{

  if ( sdHandle->Instance == SDMMC2 )
  {
    /* USER CODE BEGIN SDMMC2_MspDeInit 0 */

    /* USER CODE END SDMMC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC2_CLK_DISABLE();

    /**SDMMC2 GPIO Configuration
     PB14     ------> SDMMC2_D0
     PD6     ------> SDMMC2_CK
     PD7     ------> SDMMC2_CMD
     */
    HAL_GPIO_DeInit (GPIOB, GPIO_PIN_14);

    HAL_GPIO_DeInit (GPIOD, GPIO_PIN_6 | GPIO_PIN_7);

    /* SDMMC2 interrupt Deinit */
    HAL_NVIC_DisableIRQ (SDMMC2_IRQn);
    /* USER CODE BEGIN SDMMC2_MspDeInit 1 */

    /* USER CODE END SDMMC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
bool sdmmc2_init ( void )
{
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  if ( HAL_SD_Init (&hsd2) != HAL_OK )
  {
    return false;
  }

  return true;
}
/* USER CODE END 1 */

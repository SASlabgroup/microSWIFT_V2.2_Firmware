/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
static bool spi1_init_status = false;
static bool spi2_init_status = false;
static bool spi3_init_status = false;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInit.Spi1ClockSelection = RCC_SPI1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int32_t spi1_init ( void )
{
  int32_t ret = SPI_OK;
  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct =
    { 0 };

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_LOW;

  if ( HAL_SPI_Init (&hspi1) != HAL_OK )
  {
    ret = SPI_ERROR;
    spi1_init_status = false;
    return ret;
  }

  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;

  if ( HAL_SPIEx_SetConfigAutonomousMode (&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK )
  {
    ret = SPI_ERROR;
    spi1_init_status = false;
    return ret;
  }

  spi1_init_status = true;

  return ret;
}

//int32_t spi2_init ( void )
//{
//  int32_t ret = SPI_OK;
//  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct =
//    { 0 };
//
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_MASTER;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 0x7;
//  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
//  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
//  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
//
//  if ( HAL_SPI_Init (&hspi2) != HAL_OK )
//  {
//    ret = SPI_ERROR;
//    spi2_init_status = false;
//    return ret;
//  }
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
//  if ( HAL_SPIEx_SetConfigAutonomousMode (&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK )
//  {
//    ret = SPI_ERROR;
//    spi2_init_status = false;
//    return ret;
//  }
//
//  spi2_init_status = true;
//
//  return ret;
//}
//
//int32_t spi3_init ( void )
//{
//  int32_t ret = SPI_OK;
//  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct =
//    { 0 };
//
//  hspi3.Instance = SPI3;
//  hspi3.Init.Mode = SPI_MODE_MASTER;
//  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi3.Init.NSS = SPI_NSS_SOFT;
//  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi3.Init.CRCPolynomial = 0x7;
//  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
//  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
//  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
//
//  if ( HAL_SPI_Init (&hspi3) != HAL_OK )
//  {
//    ret = SPI_ERROR;
//    spi3_init_status = false;
//    return ret;
//  }
//
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
//  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
//
//  if ( HAL_SPIEx_SetConfigAutonomousMode (&hspi3, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK )
//  {
//    ret = SPI_ERROR;
//    spi3_init_status = false;
//    return ret;
//  }
//
//  spi3_init_status = true;
//
//  return ret;
//}

int32_t spi1_deinit ( void )
{
  int32_t ret = SPI_OK;

  if ( spi1_init_status )
  {
    if ( HAL_SPI_DeInit (&hspi1) != HAL_OK )
    {
      ret = SPI_ERROR;
    }

    spi1_init_status = false;
  }

  return ret;
}
//
//int32_t spi2_deinit ( void )
//{
//  int32_t ret = SPI_OK;
//
//  if ( spi2_init_status )
//  {
//    if ( HAL_SPI_DeInit (&hspi2) != HAL_OK )
//    {
//      ret = SPI_ERROR;
//    }
//
//    spi2_init_status = false;
//  }
//
//  return ret;
//}
//
//int32_t spi3_deinit ( void )
//{
//  int32_t ret = SPI_OK;
//
//  if ( spi3_init_status )
//  {
//    if ( HAL_SPI_DeInit (&hspi3) != HAL_OK )
//    {
//      ret = SPI_ERROR;
//    }
//
//    spi3_init_status = false;
//  }
//
//  return ret;
//}

bool spi_bus_init_status ( SPI_TypeDef *instance )
{
  if ( instance == SPI1 )
  {
    return spi1_init_status;
  }
  if ( instance == SPI2 )
  {
    return spi2_init_status;
  }
  if ( instance == SPI3 )
  {
    return spi3_init_status;
  }

  return false;
}

/* USER CODE END 1 */

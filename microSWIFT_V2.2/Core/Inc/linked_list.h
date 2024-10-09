/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : linked_list.h
 * Description        : This file provides code for the configuration
 *                      of the LinkedList.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#ifndef LINKED_LIST_H
#define LINKED_LIST_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern DMA_NodeTypeDef gnss_dma_linked_list_node;
extern DMA_QListTypeDef gnss_dma_linked_list;
/* USER CODE END ET */
/* Exported constants --------------------------------------------------------*/
HAL_StatusTypeDef MX_gnss_dma_linked_list_Config ( void );

#ifdef __cplusplus
}
#endif

#endif /* LINKED_LIST_H */


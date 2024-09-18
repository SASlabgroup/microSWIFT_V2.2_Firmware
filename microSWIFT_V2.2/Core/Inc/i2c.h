/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.h
 * @brief   This file contains all the function prototypes for
 *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c2;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */
#define I2C_OK 0
#define I2C_ERROR -1
/* USER CODE END Private defines */

void MX_I2C1_Init ( void );
void MX_I2C2_Init ( void );
void MX_I2C3_Init ( void );

/* USER CODE BEGIN Prototypes */
int32_t i2c1_init ( void );
int32_t i2c2_init ( void );
int32_t i2c3_init ( void );

int32_t i2c1_deinit ( void );
int32_t i2c2_deinit ( void );
int32_t i2c3_deinit ( void );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */


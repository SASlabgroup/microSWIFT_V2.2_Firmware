/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct
{
  uint8_t major_rev :4;
  uint8_t minor_rev :4;
} microSWIFT_firmware_version_t;

extern microSWIFT_firmware_version_t firmware_version;
#define COMPILE_TIME_DATE_BUFFER_SIZE 32
extern char compile_date[COMPILE_TIME_DATE_BUFFER_SIZE];
extern char compile_time[COMPILE_TIME_DATE_BUFFER_SIZE];
/* USER CODE END PV */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CT_FET_Pin GPIO_PIN_2
#define CT_FET_GPIO_Port GPIOE
#define VBATT_ADC_IN_Pin GPIO_PIN_0
#define VBATT_ADC_IN_GPIO_Port GPIOC
#define RTC_TIMESTAMP_4_Pin GPIO_PIN_1
#define RTC_TIMESTAMP_4_GPIO_Port GPIOC
#define RTC_TIMESTAMP_2_Pin GPIO_PIN_3
#define RTC_TIMESTAMP_2_GPIO_Port GPIOC
#define RTC_TIMESTAMP_3_Pin GPIO_PIN_0
#define RTC_TIMESTAMP_3_GPIO_Port GPIOB
#define SPI2_CSn1_Pin GPIO_PIN_1
#define SPI2_CSn1_GPIO_Port GPIOB
#define RTC_INT_B_Pin GPIO_PIN_2
#define RTC_INT_B_GPIO_Port GPIOB
#define RTC_INT_B_EXTI_IRQn EXTI2_IRQn
#define SPI1_CSn1_Pin GPIO_PIN_11
#define SPI1_CSn1_GPIO_Port GPIOF
#define WDOG_OR_INPUT_Pin GPIO_PIN_13
#define WDOG_OR_INPUT_GPIO_Port GPIOF
#define SPI1_CSn3_Pin GPIO_PIN_14
#define SPI1_CSn3_GPIO_Port GPIOF
#define SPI2_CSn3_Pin GPIO_PIN_15
#define SPI2_CSn3_GPIO_Port GPIOF
#define AS7341_INT_Pin GPIO_PIN_1
#define AS7341_INT_GPIO_Port GPIOG
#define SPARE_GPIO2_Pin GPIO_PIN_7
#define SPARE_GPIO2_GPIO_Port GPIOE
#define SPARE_GPIO3_Pin GPIO_PIN_8
#define SPARE_GPIO3_GPIO_Port GPIOE
#define SPARE_GPIO0_Pin GPIO_PIN_9
#define SPARE_GPIO0_GPIO_Port GPIOE
#define SPARE_GPIO4_Pin GPIO_PIN_10
#define SPARE_GPIO4_GPIO_Port GPIOE
#define ACCELEROMETER_FET_Pin GPIO_PIN_11
#define ACCELEROMETER_FET_GPIO_Port GPIOE
#define SPARE_GPIO1_Pin GPIO_PIN_12
#define SPARE_GPIO1_GPIO_Port GPIOE
#define SPI3_CSn1_Pin GPIO_PIN_13
#define SPI3_CSn1_GPIO_Port GPIOE
#define SPI3_CSn2_Pin GPIO_PIN_14
#define SPI3_CSn2_GPIO_Port GPIOE
#define UART_LOGGER_EN_Pin GPIO_PIN_15
#define UART_LOGGER_EN_GPIO_Port GPIOE
#define EXT_LED_RED_Pin GPIO_PIN_8
#define EXT_LED_RED_GPIO_Port GPIOD
#define EXT_LED_GREEN_Pin GPIO_PIN_10
#define EXT_LED_GREEN_GPIO_Port GPIOD
#define RTC_INT_A_Pin GPIO_PIN_12
#define RTC_INT_A_GPIO_Port GPIOD
#define RTC_TIMESTAMP_1_Pin GPIO_PIN_13
#define RTC_TIMESTAMP_1_GPIO_Port GPIOD
#define RTC_SPI_CS_Pin GPIO_PIN_14
#define RTC_SPI_CS_GPIO_Port GPIOD
#define IRIDIUM_FET_Pin GPIO_PIN_2
#define IRIDIUM_FET_GPIO_Port GPIOG
#define IRIDIUM_OnOff_Pin GPIO_PIN_3
#define IRIDIUM_OnOff_GPIO_Port GPIOG
#define IRIDIUM_RI_N_Pin GPIO_PIN_0
#define IRIDIUM_RI_N_GPIO_Port GPIOD
#define IRIDIUM_RI_N_EXTI_IRQn EXTI0_IRQn
#define BUS_5V_FET_Pin GPIO_PIN_4
#define BUS_5V_FET_GPIO_Port GPIOD
#define GNSS_FET_Pin GPIO_PIN_4
#define GNSS_FET_GPIO_Port GPIOB
#define RF_SWITCH_EN_Pin GPIO_PIN_0
#define RF_SWITCH_EN_GPIO_Port GPIOE
#define RF_SWITCH_VCTL_Pin GPIO_PIN_1
#define RF_SWITCH_VCTL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

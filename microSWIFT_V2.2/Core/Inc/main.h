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
extern bool initial_config_complete;
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
#define GNSS_FET_Pin GPIO_PIN_3
#define GNSS_FET_GPIO_Port GPIOE
#define IRIDIUM_FET_Pin GPIO_PIN_4
#define IRIDIUM_FET_GPIO_Port GPIOE
#define TEMP_FET_Pin GPIO_PIN_5
#define TEMP_FET_GPIO_Port GPIOE
#define LIGHT_FET_Pin GPIO_PIN_6
#define LIGHT_FET_GPIO_Port GPIOE
#define TURBIDITY_FET_Pin GPIO_PIN_13
#define TURBIDITY_FET_GPIO_Port GPIOC
#define AUX_I2C_1_SDA_Pin GPIO_PIN_0
#define AUX_I2C_1_SDA_GPIO_Port GPIOF
#define AUX_I2C_1_SCL_Pin GPIO_PIN_1
#define AUX_I2C_1_SCL_GPIO_Port GPIOF
#define SPARE_GPIO5_Pin GPIO_PIN_2
#define SPARE_GPIO5_GPIO_Port GPIOF
#define SPARE_GPIO6_Pin GPIO_PIN_3
#define SPARE_GPIO6_GPIO_Port GPIOF
#define RAM_CLK_Pin GPIO_PIN_4
#define RAM_CLK_GPIO_Port GPIOF
#define FLASH_IO3_Pin GPIO_PIN_6
#define FLASH_IO3_GPIO_Port GPIOF
#define FLASH_IO2_Pin GPIO_PIN_7
#define FLASH_IO2_GPIO_Port GPIOF
#define FLASH_IO0_Pin GPIO_PIN_8
#define FLASH_IO0_GPIO_Port GPIOF
#define FLASH_IO1_Pin GPIO_PIN_9
#define FLASH_IO1_GPIO_Port GPIOF
#define FLASH_CLK_Pin GPIO_PIN_10
#define FLASH_CLK_GPIO_Port GPIOF
#define VBATT_ADC_IN_Pin GPIO_PIN_0
#define VBATT_ADC_IN_GPIO_Port GPIOC
#define RTC_TIMESTAMP_4_Pin GPIO_PIN_1
#define RTC_TIMESTAMP_4_GPIO_Port GPIOC
#define RTC_TIMESTAMP_2_Pin GPIO_PIN_3
#define RTC_TIMESTAMP_2_GPIO_Port GPIOC
#define IRIDIUM_UART_TX_Pin GPIO_PIN_0
#define IRIDIUM_UART_TX_GPIO_Port GPIOA
#define IRIDIUM_UART_RX_Pin GPIO_PIN_1
#define IRIDIUM_UART_RX_GPIO_Port GPIOA
#define RTC_TIMESTAMP_1_Pin GPIO_PIN_2
#define RTC_TIMESTAMP_1_GPIO_Port GPIOA
#define FLASH_NCS_Pin GPIO_PIN_4
#define FLASH_NCS_GPIO_Port GPIOA
#define CORE_SPI_SCK_Pin GPIO_PIN_5
#define CORE_SPI_SCK_GPIO_Port GPIOA
#define CORE_SPI_MISO_Pin GPIO_PIN_6
#define CORE_SPI_MISO_GPIO_Port GPIOA
#define CORE_SPI_MOSI_Pin GPIO_PIN_7
#define CORE_SPI_MOSI_GPIO_Port GPIOA
#define RTC_TIMESTAMP_3_Pin GPIO_PIN_0
#define RTC_TIMESTAMP_3_GPIO_Port GPIOB
#define SPI2_CSn1_Pin GPIO_PIN_1
#define SPI2_CSn1_GPIO_Port GPIOB
#define RTC_INT_B_Pin GPIO_PIN_2
#define RTC_INT_B_GPIO_Port GPIOB
#define RTC_INT_B_EXTI_IRQn EXTI2_IRQn
#define SPI1_CSn1_Pin GPIO_PIN_11
#define SPI1_CSn1_GPIO_Port GPIOF
#define RTC_INT_A_Pin GPIO_PIN_12
#define RTC_INT_A_GPIO_Port GPIOF
#define RTC_INT_A_EXTI_IRQn EXTI12_IRQn
#define SPI1_CSn2_Pin GPIO_PIN_13
#define SPI1_CSn2_GPIO_Port GPIOF
#define SPI1_CSn3_Pin GPIO_PIN_14
#define SPI1_CSn3_GPIO_Port GPIOF
#define SPI2_CSn3_Pin GPIO_PIN_15
#define SPI2_CSn3_GPIO_Port GPIOF
#define RAM_IO0_Pin GPIO_PIN_0
#define RAM_IO0_GPIO_Port GPIOG
#define RAM_IO1_Pin GPIO_PIN_1
#define RAM_IO1_GPIO_Port GPIOG
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
#define SPI3_CSn3_Pin GPIO_PIN_15
#define SPI3_CSn3_GPIO_Port GPIOE
#define SD_CARD_D0_Pin GPIO_PIN_14
#define SD_CARD_D0_GPIO_Port GPIOB
#define EXT_LED_RED_Pin GPIO_PIN_8
#define EXT_LED_RED_GPIO_Port GPIOD
#define EXT_LED_GREEN_Pin GPIO_PIN_10
#define EXT_LED_GREEN_GPIO_Port GPIOD
#define RTC_SPI_CS_Pin GPIO_PIN_14
#define RTC_SPI_CS_GPIO_Port GPIOD
#define LOGGER_UART_RX_Pin GPIO_PIN_8
#define LOGGER_UART_RX_GPIO_Port GPIOC
#define LOGGER_UART_TX_Pin GPIO_PIN_9
#define LOGGER_UART_TX_GPIO_Port GPIOC
#define GNSS_UART_TX_Pin GPIO_PIN_9
#define GNSS_UART_TX_GPIO_Port GPIOA
#define GNSS_UART_RX_Pin GPIO_PIN_10
#define GNSS_UART_RX_GPIO_Port GPIOA
#define RAM_NCS_Pin GPIO_PIN_12
#define RAM_NCS_GPIO_Port GPIOA
#define AUX_SPI_2_SCK_Pin GPIO_PIN_10
#define AUX_SPI_2_SCK_GPIO_Port GPIOC
#define AUX_SPI_2_MISO_Pin GPIO_PIN_11
#define AUX_SPI_2_MISO_GPIO_Port GPIOC
#define CT_UART_TX_Pin GPIO_PIN_12
#define CT_UART_TX_GPIO_Port GPIOC
#define IRIDIUM_RI_N_Pin GPIO_PIN_0
#define IRIDIUM_RI_N_GPIO_Port GPIOD
#define IRIDIUM_RI_N_EXTI_IRQn EXTI0_IRQn
#define CT_UART_RX_Pin GPIO_PIN_2
#define CT_UART_RX_GPIO_Port GPIOD
#define IRIDIUM_OnOff_Pin GPIO_PIN_3
#define IRIDIUM_OnOff_GPIO_Port GPIOD
#define BUS_5V_FET_Pin GPIO_PIN_4
#define BUS_5V_FET_GPIO_Port GPIOD
#define SD_CARD_CK_Pin GPIO_PIN_6
#define SD_CARD_CK_GPIO_Port GPIOD
#define SD_CARD_CMD_Pin GPIO_PIN_7
#define SD_CARD_CMD_GPIO_Port GPIOD
#define RAM_IO2_Pin GPIO_PIN_9
#define RAM_IO2_GPIO_Port GPIOG
#define RAM_IO3_Pin GPIO_PIN_10
#define RAM_IO3_GPIO_Port GPIOG
#define CORE_I2C_SDA_Pin GPIO_PIN_13
#define CORE_I2C_SDA_GPIO_Port GPIOG
#define CORE_I2C_SCL_Pin GPIO_PIN_14
#define CORE_I2C_SCL_GPIO_Port GPIOG
#define UART_LOGGER_EN_Pin GPIO_PIN_15
#define UART_LOGGER_EN_GPIO_Port GPIOG
#define AUX_SPI_2_MOSI_Pin GPIO_PIN_5
#define AUX_SPI_2_MOSI_GPIO_Port GPIOB
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

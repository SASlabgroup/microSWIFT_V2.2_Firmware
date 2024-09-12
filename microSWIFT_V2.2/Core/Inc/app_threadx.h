/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.h
 * @author  MCD Application Team
 * @brief   ThreadX applicative header file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if SD_CARD_ENABLED
   #include "sdmmc.h"
#endif

#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "octospi.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// @formatter:off
enum initialization_flags
{
  RTC_INIT_SUCCESS              = ((ULONG) 1 << 0),
  GNSS_INIT_SUCCESS             = ((ULONG) 1 << 1),
  CT_INIT_SUCCESS               = ((ULONG) 1 << 2),
  TEMPERATURE_INIT_SUCCESS      = ((ULONG) 1 << 3),
  TURBIDITY_INIT_SUCCESS        = ((ULONG) 1 << 4),
  LIGHT_INIT_SUCCESS            = ((ULONG) 1 << 5),
  ACCELEROMETER_INIT_SUCCESS    = ((ULONG) 1 << 6),
  WAVES_THREAD_INIT_SUCCESS     = ((ULONG) 1 << 7),
  IRIDIUM_INIT_SUCCESS          = ((ULONG) 1 << 8)
};

enum complete_flags
{
  GNSS_THREAD_COMPLETE          = ((ULONG) 1 << 0),
  CT_THREAD_COMPLETE            = ((ULONG) 1 << 1),
  TEMPERATURE_THREAD_COMPLETE   = ((ULONG) 1 << 2),
  TURBIDITY_THREAD_COMPLETE     = ((ULONG) 1 << 3),
  LIGHT_THREAD_COMPLETE         = ((ULONG) 1 << 4),
  ACCELEROMETER_THREAD_COMPLETE = ((ULONG) 1 << 5),
  WAVES_THREAD_COMPLETE         = ((ULONG) 1 << 6),
  IRIDIUM_THREAD_COMPLETE       = ((ULONG) 1 << 7),
};

enum interrupt_flags
{
  // GNSS DMA Reception flags (others use semaphores)
  GNSS_CONFIG_RECVD             = ((ULONG) 1 << 0),
  GNSS_TX_COMPLETE              = ((ULONG) 1 << 1),
  GNSS_MSG_RECEIVED             = ((ULONG) 1 << 2),
  GNSS_MSG_INCOMPLETE           = ((ULONG) 1 << 3),
  // Signal the ADC conversion has been completed
  BATTERY_CONVERSION_COMPLETE   = ((ULONG) 1 << 4),
};

typedef enum error_flags
{
  INVALID_CONFIGURATION      = ((ULONG) 1 << 0),
  RTC_ERROR                  = ((ULONG) 1 << 1),
  GNSS_ERROR                 = ((ULONG) 1 << 2),
  CT_ERROR                   = ((ULONG) 1 << 3),
  TEMPERATURE_ERROR          = ((ULONG) 1 << 4),
  LIGHT_ERROR                = ((ULONG) 1 << 5),
  TURBIDITY_ERROR            = ((ULONG) 1 << 6),
  ACCELEROMETER_ERROR        = ((ULONG) 1 << 7),
  WAVES_THREAD_ERROR         = ((ULONG) 1 << 8),
  IRIDIUM_ERROR              = ((ULONG) 1 << 9),
  FILE_SYSTEM_ERROR          = ((ULONG) 1 << 10),
  WATCHDOG_RESET             = ((ULONG) 1 << 11),
  SOFTWARE_RESET             = ((ULONG) 1 << 12),
  GNSS_RESOLUTION_ERROR      = ((ULONG) 1 << 13),
  GNSS_TOO_MANY_PARTIAL_MSGS = ((ULONG) 1 << 14),
  GNSS_SAMPLE_WINDOW_TIMEOUT = ((ULONG) 1 << 15),
  GNSS_FRAME_SYNC_FAILED     = ((ULONG) 1 << 16),
  GNSS_SAMPLE_WINDOW_ERROR   = ((ULONG) 1 << 17),
  MEMORY_CORRUPTION          = ((ULONG) 1 << 18)
} error_flags_t;

extern TX_THREAD control_thread;
extern TX_THREAD rtc_thread;
extern TX_THREAD logger_thread;
extern TX_THREAD gnss_thread;
extern TX_THREAD ct_thread;
extern TX_THREAD temperature_thread;
extern TX_THREAD light_thread;
extern TX_THREAD turbidity_thread;
extern TX_THREAD accelerometer_thread;
extern TX_THREAD waves_thread;
extern TX_THREAD iridium_thread;
extern TX_THREAD expansion_thread_1;
extern TX_THREAD expansion_thread_2;
extern TX_THREAD expansion_thread_3;

extern TX_SEMAPHORE ext_rtc_spi_sema;
extern TX_SEMAPHORE aux_spi_1_spi_sema;
extern TX_SEMAPHORE aux_spi_2_spi_sema;
extern TX_SEMAPHORE core_i2c_sema;
extern TX_SEMAPHORE aux_i2c_1_sema;
extern TX_SEMAPHORE aux_i2c_2_sema;
extern TX_SEMAPHORE iridium_uart_sema;
extern TX_SEMAPHORE ct_uart_sema;
extern TX_SEMAPHORE aux_uart_1_sema;
extern TX_SEMAPHORE aux_uart_2_sema;

extern TX_EVENT_FLAGS_GROUP initialization_flags;
extern TX_EVENT_FLAGS_GROUP irq_flags;
extern TX_EVENT_FLAGS_GROUP error_flags;

typedef struct
{
  // Core
#if SD_CARD_ENABLED
  SD_HandleTypeDef *sd_handle;
#endif
  SPI_HandleTypeDef     *core_spi_handle;
  I2C_HandleTypeDef     *core_i2c_handle;
  UART_HandleTypeDef    *iridium_uart_handle;
  UART_HandleTypeDef    *gnss_uart_handle;
  UART_HandleTypeDef    *ct_uart_handle;
  OSPI_HandleTypeDef    *ext_flash_handle;
  ADC_HandleTypeDef     *battery_adc;
  // Expansion/ spares
  SPI_HandleTypeDef     *aux_spi_1_handle;
  SPI_HandleTypeDef     *aux_spi_2_handle;
  I2C_HandleTypeDef     *aux_i2c_1_handle;
  I2C_HandleTypeDef     *aux_i2c_2_handle;
  UART_HandleTypeDef    *aux_uart_1_handle;
  UART_HandleTypeDef    *aux_uart_2_handle;
  // DMA handles
  DMA_HandleTypeDef     *gnss_uart_tx_dma_handle;
  DMA_HandleTypeDef     *gnss_uart_rx_dma_handle;
  DMA_HandleTypeDef     *iridium_uart_tx_dma_handle;
  DMA_HandleTypeDef     *iridium_uart_rx_dma_handle;
  DMA_HandleTypeDef     *ct_uart_tx_dma_handle;
  DMA_HandleTypeDef     *ct_uart_rx_dma_handle;
  DMA_HandleTypeDef     *aux_uart_1_tx_dma_handle;
  DMA_HandleTypeDef     *aux_uart_1_rx_dma_handle;
  DMA_HandleTypeDef     *aux_uart_2_tx_dma_handle;
  DMA_HandleTypeDef     *aux_uart_2_rx_dma_handle;
} Device_Handles;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define WAVES_MEM_POOL_SIZE 651264
#define EXPANSION_QUEUE_MSG_SIZE 64
#define EXPANSION_QUEUE_LENGTH 4
/* USER CODE END EC */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Main thread defines -------------------------------------------------------*/
/* USER CODE BEGIN MTD */

/* USER CODE END MTD */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define MS_PER_SECOND 1000

// The max times we'll try to get a single peripheral up before sending reset vector
#define MAX_SELF_TEST_RETRIES 10
// The maximum amount of time (in milliseconds) a sample window could take
#define MAX_ALLOWABLE_WINDOW_TIME_IN_MINUTES 61
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* USER CODE BEGIN 1 */
// @formatter:on
/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */

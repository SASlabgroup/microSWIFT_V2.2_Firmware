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
#include "threadx_support.h"
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

enum rtc_server_complete_flags
{
  CONTROL_REQUEST_COMPLETE          = ((ULONG) 1 << 0),
  GNSS_REQUEST_COMPLETE             = ((ULONG) 1 << 1),
  CT_REQUEST_COMPLETE               = ((ULONG) 1 << 2),
  TEMPERATURE_REQUEST_COMPLETE      = ((ULONG) 1 << 3),
  LIGHT_REQUEST_COMPLETE            = ((ULONG) 1 << 4),
  TURBIDITY_REQUEST_COMPLETE        = ((ULONG) 1 << 5),
  ACCELEROMETER_REQUEST_COMPLETE    = ((ULONG) 1 << 6),
  WAVES_REQUEST_COMPLETE            = ((ULONG) 1 << 7),
  IRIDIUM_REQUEST_COMPLETE          = ((ULONG) 1 << 8),
};

enum thread_complete_flags
{
  GNSS_THREAD_COMPLETED_SUCCESSFULLY            = ((ULONG) 1 << 0),
  GNSS_THREAD_COMPLETED_WITH_ERRORS             = ((ULONG) 1 << 1),
  GNSS_TWO_MINS_OUT_FROM_COMPLETION             = ((ULONG) 1 << 2),
  CT_THREAD_COMPLETED_SUCCESSFULLY              = ((ULONG) 1 << 3),
  CT_THREAD_COMPLETED_WITH_ERRORS               = ((ULONG) 1 << 4),
  TEMPERATURE_THREAD_COMPLETED_SUCCESSFULLY     = ((ULONG) 1 << 5),
  TEMPERATURE_THREAD_COMPLETED_WITH_ERRORS      = ((ULONG) 1 << 6),
  TURBIDITY_THREAD_COMPLETED_SUCCESSFULLY       = ((ULONG) 1 << 7),
  TURBIDITY_THREAD_COMPLETED_WITH_ERRORS        = ((ULONG) 1 << 8),
  LIGHT_THREAD_COMPLETED_SUCCESSFULLY           = ((ULONG) 1 << 9),
  LIGHT_THREAD_COMPLETED_WITH_ERRORS            = ((ULONG) 1 << 10),
  ACCELEROMETER_THREAD_COMPLETED_SUCCESSFULLY   = ((ULONG) 1 << 11),
  ACCELEROMETER_THREAD_COMPLETED_WITH_ERRORS    = ((ULONG) 1 << 12),
  WAVES_THREAD_COMPLETED_SUCCESSFULLY           = ((ULONG) 1 << 13),
  WAVES_THREAD_COMPLETED_WITH_ERRORS            = ((ULONG) 1 << 14),
  IRIDIUM_THREAD_COMPLETED_SUCCESSFULLY         = ((ULONG) 1 << 15),
  IRIDIUM_THREAD_COMPLETED_WITH_ERRORS          = ((ULONG) 1 << 16),
};

enum interrupt_flags
{
  // GNSS DMA flags (others use semaphores)
  GNSS_CONFIG_RECVD             = ((ULONG) 1 << 0),
  GNSS_TX_COMPLETE              = ((ULONG) 1 << 1),
  GNSS_MSG_RECEIVED             = ((ULONG) 1 << 2),
  GNSS_MSG_INCOMPLETE           = ((ULONG) 1 << 3),
  // Signal the ADC conversion has been completed
  BATTERY_CONVERSION_COMPLETE   = ((ULONG) 1 << 4),
};

typedef enum error_flags
{
  // Thread/ sensor errors
  GNSS_INIT_FAILED                      = ((ULONG) 1 << 0),
  GNSS_CONFIGURATION_FAILED             = ((ULONG) 1 << 1),
  GNSS_RESOLUTION_ERROR                 = ((ULONG) 1 << 2),
  GNSS_TOO_MANY_PARTIAL_MSGS            = ((ULONG) 1 << 3),
  GNSS_SAMPLE_WINDOW_TIMEOUT            = ((ULONG) 1 << 4),
  GNSS_FRAME_SYNC_FAILED                = ((ULONG) 1 << 5),
  GNSS_SAMPLE_WINDOW_ERROR              = ((ULONG) 1 << 6),
  CT_INIT_FAILED                        = ((ULONG) 1 << 7),
  CT_SAMPLING_ERROR                     = ((ULONG) 1 << 8),
  CT_SAMPLE_WINDOW_TIMEOUT              = ((ULONG) 1 << 9),
  TEMPERATURE_INIT_FAILED               = ((ULONG) 1 << 10),
  TEMPERATURE_SAMPLING_ERROR            = ((ULONG) 1 << 11),
  TEMPERATURE_SAMPLE_WINDOW_TIMEOUT     = ((ULONG) 1 << 12),
  LIGHT_INIT_FAILED                     = ((ULONG) 1 << 13),
  LIGHT_SAMPLING_ERROR                  = ((ULONG) 1 << 14),
  LIGHT_SAMPLE_WINDOW_TIMEOUT           = ((ULONG) 1 << 15),
  TURBIDITY_INIT_FAILED                 = ((ULONG) 1 << 16),
  TURBIDITY_SAMPLING_ERROR              = ((ULONG) 1 << 17),
  TURBIDITY_SAMPLE_WINDOW_TIMEOUT       = ((ULONG) 1 << 18),
  ACCELEROMETER_INIT_FAILED             = ((ULONG) 1 << 19),
  ACCELEROMETER_SAMPLING_ERROR          = ((ULONG) 1 << 20),
  ACCELEROMETER_SAMPLE_WINDOW_TIMEOUT   = ((ULONG) 1 << 21),
  IRIDIUM_INIT_ERROR                    = ((ULONG) 1 << 22),
  IRIDIUM_UART_COMMS_ERROR              = ((ULONG) 1 << 23),
  FILE_SYSTEM_ERROR                     = ((ULONG) 1 << 24),
  RTC_ERROR                             = ((ULONG) 1 << 25),
  WAVES_INIT_FAILED                     = ((ULONG) 1 << 26),
  // Misc errors
  WATCHDOG_RESET                        = ((ULONG) 1 << 29),
  SOFTWARE_RESET                        = ((ULONG) 1 << 30),
  MEMORY_CORRUPTION_ERROR               = ((ULONG) 1 << 31)
} error_flags_t;

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
  UART_HandleTypeDef    *logger_uart_handle;
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

typedef struct
{
  TX_THREAD *control_thread;
  TX_THREAD *rtc_thread;
  TX_THREAD *logger_thread;
  TX_THREAD *gnss_thread;
  TX_THREAD *ct_thread;
  TX_THREAD *temperature_thread;
  TX_THREAD *light_thread;
  TX_THREAD *turbidity_thread;
  TX_THREAD *waves_thread;
  TX_THREAD *iridium_thread;
  TX_THREAD *filex_thread;
} Thread_Handles;

extern Thread_Handles thread_handles;
extern Device_Handles device_handles;

extern TX_SEMAPHORE ext_rtc_spi_sema;
extern TX_SEMAPHORE aux_spi_1_spi_sema;
extern TX_SEMAPHORE aux_spi_2_spi_sema;
extern TX_SEMAPHORE light_sensor_i2c_sema;
extern TX_SEMAPHORE turbidity_sensor_i2c_sema;
extern TX_SEMAPHORE aux_i2c_1_sema;
extern TX_SEMAPHORE aux_i2c_2_sema;
extern TX_SEMAPHORE iridium_uart_sema;
extern TX_SEMAPHORE ct_uart_sema;
extern TX_SEMAPHORE aux_uart_1_sema;
extern TX_SEMAPHORE aux_uart_2_sema;
extern TX_SEMAPHORE logger_sema;
extern TX_SEMAPHORE light_sensor_int_pin_sema;

extern TX_EVENT_FLAGS_GROUP initialization_flags;
extern TX_EVENT_FLAGS_GROUP irq_flags;
extern TX_EVENT_FLAGS_GROUP error_flags;
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

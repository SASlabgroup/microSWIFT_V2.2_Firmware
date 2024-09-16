/*
 * CT.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_CT_H_
#define SRC_CT_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "generic_uart_driver.h"
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include "configuration.h"

// @formatter:off

#ifdef DEBUGGING_FAST_CYCLE
#define WARMUP_TIME                     20
#else
#define WARMUP_TIME                     20000
#endif
// The total length of a response sentence from the sensor
#define CT_DATA_ARRAY_SIZE              291
#define TEMP_MEASUREMENT_START_INDEX    70
#define TEMP_OFFSET_FROM_UNITS          6
#define SALINITY_OFFSET_FROM_UNITS      4
#define SAMPLE_TIME_IN_MILLISECONDS     2000
#define CT_VALUES_ERROR_CODE            0x70E2
#define CT_UART_TX_TIMEOUT_TICKS        (TX_TIMER_TICKS_PER_SECOND / 10)
#define CT_UART_RX_TIMEOUT_TICKS        (TX_TIMER_TICKS_PER_SECOND * 3)

typedef enum ct_error_code
{
  CT_SUCCESS            =  0,
  CT_UART_ERROR         = -1,
  CT_PARSING_ERROR      = -2,
  CT_SELF_TEST_FAIL     = -3,
  CT_NOT_ENOUGH_SAMPLES = -4,
  CT_DONE_SAMPLING      = -5
} ct_return_code_t;

typedef struct
{
  double salinity;
  double temp;
} ct_sample;

typedef struct CT
{
  // Our global configuration struct
  microSWIFT_configuration  *global_config;
  generic_uart_driver       uart_driver;
  // The UART and DMA handle for the CT interface
  UART_HandleTypeDef        *uart_handle;
  DMA_HandleTypeDef         *tx_dma_handle;
  DMA_HandleTypeDef         *rx_dma_handle;
  // Event flags
  TX_EVENT_FLAGS_GROUP      *error_flags;
  // The buffer written to by CT sensor
  char                      data_buf[CT_DATA_ARRAY_SIZE];
  // Arrays to hold conductivity/temp values
  ct_sample                 samples_accumulator;
  ct_sample                 samples_averages;
  // Track the number of samples
  int32_t                   total_samples;
  // Timer timeout
  bool                      timer_timeout;
  // Function pointers
  ct_return_code_t          (*parse_sample) ( void );
  ct_return_code_t          (*get_averages) ( void );
  ct_return_code_t          (*self_test) ( bool add_warmup_time, ct_sample *optional_readings );
  ct_return_code_t          (*uart_init) ( void );
  ct_return_code_t          (*reset_ct_uart) ( void );
  void                      (*on) ( void );
  void                      (*off) ( void );
} CT;

void ct_init ( CT *struct_ptr, microSWIFT_configuration *global_config,
               UART_HandleTypeDef *ct_uart_handle,  DMA_HandleTypeDef *ct_tx_dma_handle,
               DMA_HandleTypeDef *ct_rx_dma_handle, TX_SEMAPHORE *uart_sema,
               TX_EVENT_FLAGS_GROUP *error_flags);

void ct_deinit ( void );

void ct_timer_timeout ( void );
bool ct_get_timeout_status ( void );
// @formatter:on
#endif /* SRC_CT_H_ */

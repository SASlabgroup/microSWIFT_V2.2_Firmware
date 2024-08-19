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
#include "stdint.h"
#include "configuration.h"

#ifdef DEBUGGING_FAST_CYCLE
#define WARMUP_TIME 20
#else
#define WARMUP_TIME 20000
#endif
// The total length of a response sentence from the sensor
#define CT_DATA_ARRAY_SIZE 291
#define CT_DEFAULT_BAUD_RATE 9600
#define TEMP_MEASUREMENT_START_INDEX 70
#define TEMP_OFFSET_FROM_UNITS 6
#define SALINITY_OFFSET_FROM_UNITS 4
#define SAMPLE_TIME_IN_MILLISECONDS 2000
#define MAX_RETRIES 10
#define CT_AVERAGED_VALUE_ERROR_CODE 0x70E2

typedef enum ct_error_code
{
  CT_SUCCESS = 0,
  CT_UART_ERROR = -1,
  CT_PARSING_ERROR = -2,
  CT_SELF_TEST_FAIL = -3,
  CT_NOT_ENOUGH_SAMPLES = -4,
  CT_DONE_SAMPLING = -5
} ct_error_code_t;

typedef struct ct_samples
{
  double salinity;
  double temp;
} ct_samples;

typedef struct CT
{
  // Our global configuration struct
  microSWIFT_configuration *global_config;
  generic_uart_driver uart_driver;
  // Event flags
  TX_EVENT_FLAGS_GROUP *control_flags;
  TX_EVENT_FLAGS_GROUP *error_flags;
  // The buffer written to by CT sensor
  char *data_buf;
  // Arrays to hold conductivity/temp values
  ct_samples *samples_buf;
  ct_samples averages;
  // Keep track of the number of samples
  uint32_t total_samples;
  // Function pointers
  ct_error_code_t (*parse_sample) ( void );
  ct_error_code_t (*get_averages) ( void );
  void (*on_off) ( GPIO_PinState pin_state );
  ct_error_code_t (*self_test) ( bool add_warmup_time );
  ct_error_code_t (*reset_ct_uart) ( void );
} CT;

ct_error_code_t ct_init ( CT *struct_ptr, microSWIFT_configuration *global_config,
                          UART_HandleTypeDef *ct_uart_handle, TX_EVENT_FLAGS_GROUP *control_flags,
                          TX_EVENT_FLAGS_GROUP *error_flags, char *data_buf,
                          ct_samples *samples_buf );

#endif /* SRC_CT_H_ */

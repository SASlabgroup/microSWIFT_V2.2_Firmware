/*
 * CT.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "generic_uart_driver.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"
#include "configuration.h"
#include "ct_sensor.h"
#include "stdarg.h"

// @formatter:off
// Object instance pointer
CT *self;

static ct_return_code_t _ct_parse_sample ( void );
static ct_return_code_t _ct_get_averages ( ct_sample *readings );
static ct_return_code_t _ct_self_test ( bool add_warmup_time, ct_sample *optional_readings );
static ct_return_code_t _ct_uart_init ( void );
static ct_return_code_t _ct_reset_uart ( void );
static ct_return_code_t _ct_start_timer ( uint16_t timeout_in_minutes );
static ct_return_code_t _ct_stop_timer ( void );
static void             _ct_on ( void );
static void             _ct_off ( void );

// Helper functions
static void             __reset_ct_struct_fields ( void );

// Search terms
static const char *temp_units =     "Deg.C";
static const char *salinity_units = "PSU";

// @formatter:on

/**
 * Initialize the CT struct
 *
 * @return void
 */
void ct_init ( CT *struct_ptr, microSWIFT_configuration *global_config,
               UART_HandleTypeDef *ct_uart_handle, DMA_HandleTypeDef *ct_tx_dma_handle,
               DMA_HandleTypeDef *ct_rx_dma_handle, TX_SEMAPHORE *uart_sema,
               TX_EVENT_FLAGS_GROUP *error_flags )
{
  // Assign object pointer
  self = struct_ptr;

  __reset_ct_struct_fields ();
  self->global_config = global_config;
  self->error_flags = error_flags;
  self->uart_handle = ct_uart_handle;
  self->tx_dma_handle = ct_tx_dma_handle;
  self->rx_dma_handle = ct_rx_dma_handle;
  self->parse_sample = _ct_parse_sample;
  self->get_averages = _ct_get_averages;
  self->self_test = _ct_self_test;
  self->uart_init = _ct_uart_init;
  self->reset_ct_uart = _ct_reset_uart;
  self->start_timer = _ct_start_timer;
  self->stop_timer = _ct_stop_timer;
  self->on = _ct_on;
  self->off = _ct_off;

  generic_uart_register_io_functions (&self->uart_driver, ct_uart_handle, uart_sema, uart5_init,
                                      uart5_deinit, NULL, NULL);
  generic_uart_set_timeout_ticks (&self->uart_driver, CT_UART_TX_TIMEOUT_TICKS,
                                  CT_UART_RX_TIMEOUT_TICKS);
}

/**
 * Deinitialize the CT UART and DMA channels
 *
 * @return void
 */
void ct_deinit ( void )
{
  self->uart_driver.deinit ();
  HAL_DMA_DeInit (self->tx_dma_handle);
  HAL_DMA_DeInit (self->rx_dma_handle);
}

/**
 * Timer timeout callback
 *
 * @return void
 */
void ct_timer_expired ( ULONG expiration_input )
{
  (void) expiration_input;
  self->timer_timeout = true;
}

/**
 * Timer timeout callback
 *
 * @return void
 */
bool ct_get_timeout_status ( void )
{
  return self->timer_timeout;
}

/**
 *
 *
 * @return ct_return_code_t
 */
static ct_return_code_t _ct_parse_sample ( void )
{
  ct_return_code_t return_code = CT_SUCCESS;
  int fail_counter = 0, max_retries = 10;
  double temperature, salinity;
  char *index;
  // Sensor sends a message every 2 seconds @ 9600 baud, takes 0.245 seconds to get it out
  int required_ticks_to_get_message = TX_TIMER_TICKS_PER_SECOND * 3;

  // Samples array overflow safety check
  if ( self->total_samples >= self->global_config->total_ct_samples )
  {
    return_code = CT_DONE_SAMPLING;
    return return_code;
  }

  while ( ++fail_counter < max_retries )
  {

    if ( generic_uart_read (&self->uart_driver, (uint8_t*) &(self->data_buf[0]),
    CT_DATA_ARRAY_SIZE,
                            required_ticks_to_get_message)
         != UART_OK )
    {
      self->reset_ct_uart ();
      return_code = CT_UART_ERROR;
      continue;
    }

    index = strstr (self->data_buf, temp_units);
    // Make the message was received in the right alignment
    if ( index == NULL || index > &(self->data_buf[0]) + TEMP_MEASUREMENT_START_INDEX )
    {
      // If this evaluates to true, we're out of sync. Insert a short delay
      return_code = CT_PARSING_ERROR;
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
      continue;
    }
    index += TEMP_OFFSET_FROM_UNITS;
    temperature = atof (index);
    // error return of atof() is 0.0
    if ( temperature == 0.0 )
    {
      continue;
    }

    char *index = strstr (self->data_buf, salinity_units);
    if ( index == NULL )
    {
      continue;
    }

    index += SALINITY_OFFSET_FROM_UNITS;
    salinity = atof (index);

    if ( salinity == 0.0 )
    {
      continue;
    }

    self->samples_accumulator.salinity += salinity;
    self->samples_accumulator.temp += temperature;

    self->total_samples++;

    return_code = CT_SUCCESS;
    break;
  }

  return return_code;
}

/**
 *
 *
 * @return ct_samples struct containing the averages conductivity
 *         and temperature values
 */
static ct_return_code_t _ct_get_averages ( ct_sample *readings )
{
  if ( self->total_samples < self->global_config->total_ct_samples )
  {
    return CT_NOT_ENOUGH_SAMPLES;
  }

  self->samples_averages.temp = self->samples_accumulator.salinity / ((double) self->total_samples);
  self->samples_averages.salinity = self->samples_accumulator.temp / ((double) self->total_samples);

  readings->temp = self->samples_averages.temp;
  readings->salinity = self->samples_averages.salinity;

  return CT_SUCCESS;
}

/**
 *
 *
 * @return ct_return_code_t
 */
static ct_return_code_t _ct_self_test ( bool add_warmup_time, ct_sample *optional_readings )
{
  ct_return_code_t return_code;
  uint32_t elapsed_time, start_time;
  double temperature, salinity;
  char *index;
  // Sensor sends a message every 2 seconds @ 9600 baud, takes 0.245 seconds to get it out
  int required_ticks_to_get_message = TX_TIMER_TICKS_PER_SECOND * 3;

  start_time = tx_time_get ();

  if ( generic_uart_read (&self->uart_driver, (uint8_t*) &(self->data_buf[0]), CT_DATA_ARRAY_SIZE,
                          required_ticks_to_get_message)
       != UART_OK )
  {
    self->reset_ct_uart ();
    return_code = CT_UART_ERROR;
    return return_code;
  }

  index = strstr (self->data_buf, temp_units);
  // Make the message was received in the right alignment
  if ( index == NULL || index > &(self->data_buf[0]) + TEMP_MEASUREMENT_START_INDEX )
  {
    return_code = CT_PARSING_ERROR;
    return return_code;
  }
  index += TEMP_OFFSET_FROM_UNITS;
  temperature = atof (index);
  // error return of atof() is 0.0
  if ( temperature == 0.0 )
  {
    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  index = strstr (self->data_buf, salinity_units);
  if ( index == NULL )
  {
    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  index += SALINITY_OFFSET_FROM_UNITS;
  salinity = atof (index);

  if ( salinity == 0.0 )
  {
    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  if ( add_warmup_time )
  {
    // Handle the warmup delay
    elapsed_time = tx_time_get () - start_time;
    int32_t required_delay = WARMUP_TIME - elapsed_time;
    if ( required_delay > 0 )
    {
      tx_thread_sleep ((required_delay / MS_PER_SECOND) * TX_TIMER_TICKS_PER_SECOND);
    }
  }

  if ( optional_readings != NULL )
  {
    optional_readings->salinity = salinity;
    optional_readings->temp = temperature;
  }

  return_code = CT_SUCCESS;
  return return_code;

}

static ct_return_code_t _ct_uart_init ( void )
{
  return (self->uart_driver.init () == UART_OK) ?
      CT_SUCCESS : CT_UART_ERROR;
}

/**
 * Reinitialize the CT UART port.
 *
 * @param self - GNSS struct
 * @param baud_rate - baud rate to set port to
 */
static ct_return_code_t _ct_reset_uart ( void )
{
  if ( self->uart_driver.deinit () != UART_OK )
  {
    return CT_UART_ERROR;
  }

  tx_thread_sleep (1);

  return _ct_uart_init ();
}

/**
 * Start the CT thread timer
 *
 * @param timeout_in_minutes - timeout, in minutes
 * @return  ct_return_code_t
 */
static ct_return_code_t _ct_start_timer ( uint16_t timeout_in_minutes );
static ct_return_code_t _ct_stop_timer ( void );

/**
 * Turn on the CT sensor FET.
 *
 * @return Void
 */
static void _ct_on ( void )
{
  HAL_GPIO_WritePin (GPIOG, CT_FET_Pin, GPIO_PIN_SET);
}

/**
 * Turn off the CT sensor FET.
 *
 * @return Void
 */
static void _ct_off ( void )
{
  HAL_GPIO_WritePin (GPIOG, CT_FET_Pin, GPIO_PIN_RESET);
}

static void __reset_ct_struct_fields ( void )
{
  self->timer_timeout = false;

  self->samples_accumulator.salinity = 0.0f;
  self->samples_accumulator.temp = 0.0f;
  // We will know if the CT sensor fails by the value 9999 in the iridium message
  self->samples_averages.salinity = CT_VALUES_ERROR_CODE;
  self->samples_averages.temp = CT_VALUES_ERROR_CODE;

  self->total_samples = 0;

  // zero out the buffer
  memset (&(self->data_buf[0]), 0, CT_DATA_ARRAY_SIZE);
}

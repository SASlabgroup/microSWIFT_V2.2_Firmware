/*
 * CT.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "ct_sensor.h"

// Object instance pointer
CT *self;

static ct_error_code_t ct_parse_sample ( void );
static ct_error_code_t ct_get_averages ( void );
static void ct_on_off ( GPIO_PinState pin_state );
static ct_error_code_t ct_self_test ( bool add_warmup_time );
static ct_error_code_t reset_ct_uart ( void );

// Helper functions
static void reset_ct_struct_fields ( void );

// Search terms
static const char *temp_units = "Deg.C";
static const char *salinity_units = "PSU";

/**
 * Initialize the CT struct
 *
 * @return void
 */
ct_error_code_t ct_init ( CT *struct_ptr, microSWIFT_configuration *global_config,
                          UART_HandleTypeDef *ct_uart_handle, DMA_HandleTypeDef *ct_dma_handle,
                          TX_EVENT_FLAGS_GROUP *control_flags, TX_EVENT_FLAGS_GROUP *error_flags,
                          char *data_buf, ct_samples *samples_buf )
{
  // Assign object pointer
  self = struct_ptr;

  reset_ct_struct_fields ();
  self->global_config = global_config;
  self->control_flags = control_flags;
  self->error_flags = error_flags;
  self->data_buf = data_buf;
  self->samples_buf = samples_buf;
  self->parse_sample = ct_parse_sample;
  self->get_averages = ct_get_averages;
  self->on_off = ct_on_off;
  self->self_test = ct_self_test;
  self->reset_ct_uart = reset_ct_uart;
  // zero out the buffer
  memset (&(self->data_buf[0]), 0, CT_DATA_ARRAY_SIZE);
  memset (&(self->samples_buf[0]), 0, self->global_config->total_ct_samples * sizeof(ct_samples));

  return generic_uart_init (&self->uart_driver, &huart5, &ct_uart_sema, uart5_init, uart5_deinit);
}

/**
 *
 *
 * @return ct_error_code_t
 */
static ct_error_code_t ct_parse_sample ( void )
{
  ULONG actual_flags;
  ct_error_code_t return_code = CT_SUCCESS;
  int fail_counter = 0;
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

  while ( ++fail_counter < MAX_RETRIES )
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

    self->samples_buf[self->total_samples].salinity = salinity;
    self->samples_buf[self->total_samples].temp = temperature;
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
static ct_error_code_t ct_get_averages ( void )
{
  double temp_sum = 0.0;
  double salinity_sum = 0.0;

  if ( self->total_samples < self->global_config->total_ct_samples )
  {
    return CT_NOT_ENOUGH_SAMPLES;
  }

  for ( int i = 0; i < self->total_samples; i++ )
  {
    temp_sum += self->samples_buf[i].temp;
    salinity_sum += self->samples_buf[i].salinity;
  }

  self->averages.temp = temp_sum / ((double) self->total_samples);
  self->averages.salinity = salinity_sum / ((double) self->total_samples);

  return CT_SUCCESS;
}

/**
 *
 *
 * @return ct_error_code_t
 */
static void ct_on_off ( GPIO_PinState pin_state )
{
  HAL_GPIO_WritePin (GPIOG, CT_FET_Pin, pin_state);
}

/**
 *
 *
 * @return ct_error_code_t
 */
static ct_error_code_t ct_self_test ( bool add_warmup_time )
{
  ULONG actual_flags;
  ct_error_code_t return_code;
  uint32_t elapsed_time, start_time;
  double temperature, salinity;
  char *index;
  // Sensor sends a message every 2 seconds @ 9600 baud, takes 0.245 seconds to get it out
  int required_ticks_to_get_message = TX_TIMER_TICKS_PER_SECOND * 3;

  self->on_off (GPIO_PIN_SET);

  start_time = HAL_GetTick ();

  if ( generic_uart_read (&self->uart_driver, (uint8_t*) &(self->data_buf[0]), CT_DATA_ARRAY_SIZE,
                          required_ticks_to_get_message)
       != UART_OK )
  {
    self->reset_ct_uart ();
    return_code = CT_UART_ERROR;
    self->reset_ct_uart ();
    return return_code;
  }

  index = strstr (self->data_buf, temp_units);
  // Make the message was received in the right alignment
  if ( index == NULL || index > &(self->data_buf[0]) + TEMP_MEASUREMENT_START_INDEX )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    return_code = CT_PARSING_ERROR;
    return return_code;
  }
  index += TEMP_OFFSET_FROM_UNITS;
  temperature = atof (index);
  // error return of atof() is 0.0
  if ( temperature == 0.0 )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  index = strstr (self->data_buf, salinity_units);
  if ( index == NULL )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  index += SALINITY_OFFSET_FROM_UNITS;
  salinity = atof (index);

  if ( salinity == 0.0 )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    return_code = CT_PARSING_ERROR;
    return return_code;
  }

  if ( add_warmup_time )
  {
    // Handle the warmup delay
    elapsed_time = HAL_GetTick () - start_time;
    int32_t required_delay = WARMUP_TIME - elapsed_time;
    if ( required_delay > 0 )
    {
      tx_thread_sleep ((required_delay / MS_PER_SECOND) * TX_TIMER_TICKS_PER_SECOND);
    }
  }

  return_code = CT_SUCCESS;
  return return_code;

}

/**
 * Reinitialize the CT UART port. Required when switching between Tx and Rx.
 *
 * @param self - GNSS struct
 * @param baud_rate - baud rate to set port to
 */
static ct_error_code_t reset_ct_uart ( void )
{
  if ( !self->uart_driver.deinit () )
  {
    return CT_UART_ERROR;
  }

  tx_thread_sleep (1);

  return self->uart_driver.init ();
}

static void reset_ct_struct_fields ( void )
{
  // We will know if the CT sensor fails by the value 9999 in the
  // iridium message
  self->averages.salinity = CT_AVERAGED_VALUE_ERROR_CODE;
  self->averages.temp = CT_AVERAGED_VALUE_ERROR_CODE;
  self->total_samples = 0;
}

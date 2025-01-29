/*
 * temp_sensor.c
 *
 * !!! MOSTLY COPPIED FROM BLUE ROBOTICS !!!
 *
 * https://github.com/bluerobotics/BlueRobotics_TSYS01_Library?tab=readme-ov-file
 *
 *  Created on: Feb 9, 2024
 *      Author: Phil
 */

#include "temp_sensor.h"
#include "main.h"
#include "stdbool.h"
#include "stm32u5xx_hal.h"
#include "configuration.h"
#include "shared_i2c_bus.h"
#include "app_threadx.h"

// @formatter:off
// Object instance pointer
static Temperature *temperature_self;

// Struct functions
static uSWIFT_return_code_t _temperature_self_test ( float *optional_reading );
static uSWIFT_return_code_t _temperature_get_readings ( bool get_single_reading, float *temperature );
static uSWIFT_return_code_t _temperature_start_timer ( uint16_t timeout_in_minutes );
static uSWIFT_return_code_t _temperature_stop_timer ( void );
// Interface IO functions
static uSWIFT_return_code_t _temperature_i2c_read ( uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_buf, uint16_t size );
static uSWIFT_return_code_t _temperature_i2c_write( uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_buf, uint16_t size );
// Helper functions
static uSWIFT_return_code_t __init_sensor ( void );
static float                __calculate_temp ( void );
static void                 __reset_struct_fields ( bool reset_calibration );
static time_t               __get_timestamp ( void );
// @formatter:on

void temperature_init ( Temperature *struct_ptr, microSWIFT_configuration *global_config,
                        TX_EVENT_FLAGS_GROUP *error_flags, TX_TIMER *timer,
                        bool clear_calibration_data )
{
  temperature_self = struct_ptr;

  temperature_self->global_config = global_config;
  temperature_self->error_flags = error_flags;
  temperature_self->timer = timer;

  temperature_self->self_test = _temperature_self_test;
  temperature_self->get_readings = _temperature_get_readings;
  temperature_self->start_timer = _temperature_start_timer;
  temperature_self->stop_timer = _temperature_stop_timer;

  __reset_struct_fields (clear_calibration_data);
}

void temperature_timer_expired ( ULONG expiration_input )
{
  temperature_self->timer_timeout = true;
}

bool temperature_get_timeout_status ( void )
{
  return temperature_self->timer_timeout;
}

static uSWIFT_return_code_t _temperature_self_test ( float *optional_reading )
{
  uSWIFT_return_code_t ret;

  ret = __init_sensor ();

  if ( ret == uSWIFT_SUCCESS )
  {
    if ( optional_reading == NULL )
    {
      return uSWIFT_SUCCESS;
    }
    else
    {
      return _temperature_get_readings (true, optional_reading);
    }
  }

  return ret;
}

static uSWIFT_return_code_t _temperature_get_readings ( bool get_single_reading,
                                                        float *temperature )
{
  uSWIFT_return_code_t return_code = uSWIFT_SUCCESS;
  uint8_t command;
  uint8_t read_data[3] =
    { 0 };
  int32_t num_readings = (get_single_reading) ?
      1 : TOTAL_TEMPERATURE_SAMPLES;

  while ( temperature_self->samples_counter < TOTAL_TEMPERATURE_SAMPLES )
  {

    command = TSYS01_ADC_TEMP_CONV;
    return_code = _temperature_i2c_write (TSYS01_ADDR, 0, &command, 0);
    if ( return_code != uSWIFT_SUCCESS )
    {
      return return_code;
    }

    // Conversion takes 8.22ms
    tx_thread_sleep (10);

    command = TSYS01_ADC_READ;

    return_code = _temperature_i2c_read (TSYS01_ADDR, command, &(read_data[0]), sizeof(read_data));
    if ( return_code != uSWIFT_SUCCESS )
    {
      return return_code;
    }

    temperature_self->D1 = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

    temperature_self->samples[temperature_self->samples_counter] = __calculate_temp ();

    temperature_self->samples_counter++;
  }

  temperature_self->averaged_temp = 0.0f;
  for ( int i = 0; i < num_readings; i++ )
  {
    temperature_self->averaged_temp += temperature_self->samples[i];
  }

  temperature_self->averaged_temp /= num_readings;

  *temperature = temperature_self->averaged_temp;

  return return_code;
}

static uSWIFT_return_code_t _temperature_start_timer ( uint16_t timeout_in_minutes )
{
  ULONG timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( tx_timer_change (temperature_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (temperature_self->timer) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
  }

  return ret;
}

static uSWIFT_return_code_t _temperature_stop_timer ( void )
{
  return (tx_timer_deactivate (temperature_self->timer) == TX_SUCCESS) ?
      uSWIFT_SUCCESS : uSWIFT_TIMER_ERROR;
}

static uSWIFT_return_code_t __init_sensor ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  uint8_t command = TSYS01_RESET;
  uint8_t read_data[2] =
    { 0 };

  if ( i2c2_init () != I2C_OK )
  {
    return false;
  }

  // Reset the TSYS01
  ret = _temperature_i2c_write (TSYS01_ADDR, 0, &command, 0);
  if ( ret != uSWIFT_SUCCESS )
  {
    return ret;
  }
  // 2.8ms boot time
  tx_thread_sleep (5);
  // Read calibration values
  for ( uint8_t i = 0; i < 8; i++ )
  {
    command = TSYS01_PROM_READ + (i * 2);

    ret = _temperature_i2c_read ( TSYS01_ADDR, command, &(read_data[0]), sizeof(read_data));
    if ( ret != uSWIFT_SUCCESS )
    {
      return ret;
    }

    temperature_self->C[i] = (float) ((read_data[0] << 8) | read_data[1]);
  }

  return ret;
}

static float __calculate_temp ( void )
{
  float temp = 0.0;
  temperature_self->adc = temperature_self->D1 / 256;
  temp = (-2.0f) * temperature_self->C[1] / 1000000000000000000000.0f
         * pow (temperature_self->adc, 4)
         + 4.0f * temperature_self->C[2] / 10000000000000000.0f * pow (temperature_self->adc, 3)
         + (-2.0f) * temperature_self->C[3] / 100000000000.0f * pow (temperature_self->adc, 2)
         + 1.0f * temperature_self->C[4] / 1000000.0f * temperature_self->adc
         + (-1.5f) * temperature_self->C[5] / 100.0f;
  return temp;
}

static void __reset_struct_fields ( bool reset_calibration )
{
  if ( reset_calibration )
  {
    memset (temperature_self->C, 0, sizeof(temperature_self->C));
  }

  memset (&(temperature_self->samples[0]), 0, sizeof(temperature_self->samples));

  temperature_self->timer_timeout = false;

  temperature_self->samples_counter = 0;

  temperature_self->averaged_temp = 0.0f;
  temperature_self->D1 = 0;
  temperature_self->adc = 0;
}

static uSWIFT_return_code_t _temperature_i2c_read ( uint8_t dev_addr, uint8_t reg_addr,
                                                    uint8_t *read_buf, uint16_t size )
{
  return shared_i2c_read (dev_addr, reg_addr, read_buf, size, TEMPERATURE_REQUEST_PROCESSED);
}

static uSWIFT_return_code_t _temperature_i2c_write ( uint8_t dev_addr, uint8_t reg_addr,
                                                     uint8_t *write_buf, uint16_t size )
{
  return shared_i2c_write (dev_addr, reg_addr, write_buf, size, TEMPERATURE_REQUEST_PROCESSED);
}

/**
 * Helper method to generate a timestamp from the RTC.
 *
 * @return timestamp as time_t
 */
static time_t __get_timestamp ( void )
{
  uSWIFT_return_code_t rtc_ret = uSWIFT_SUCCESS;
  struct tm time;

  rtc_ret = rtc_server_get_time (&time, TEMPERATURE_REQUEST_PROCESSED);
  if ( rtc_ret != uSWIFT_SUCCESS )
  {
    return -1;
  }

  return mktime (&time);
}

// @formatter:on

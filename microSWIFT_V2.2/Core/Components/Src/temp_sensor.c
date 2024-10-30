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
#include "i2c.h"

// @formatter:off
// Object instance pointer
static Temperature *temperature_self;

// Struct functions
static temperature_return_code_t _temperature_self_test ( float *optional_reading );
static temperature_return_code_t _temperature_get_readings ( bool get_single_reading, float *temperature );
static temperature_return_code_t _temperature_start_timer ( uint16_t timeout_in_minutes );
static temperature_return_code_t _temperature_stop_timer ( void );
static void                      _temperature_on ( void );
static void                      _temperature_off ( void );

// Helper functions
static bool                      __init_sensor ( void );
static float                     __calculate_temp ( void );
static void                      __reset_struct_fields ( bool reset_calibration );
// @formatter:on

void temperature_init ( Temperature *struct_ptr, microSWIFT_configuration *global_config,
                        I2C_HandleTypeDef *i2c_handle, TX_EVENT_FLAGS_GROUP *error_flags,
                        TX_TIMER *timer, TX_MUTEX *i2c_mutex, bool clear_calibration_data )
{
  temperature_self = struct_ptr;

  temperature_self->global_config = global_config;
  temperature_self->error_flags = error_flags;
  temperature_self->timer = timer;
  temperature_self->pwr_gpio.port = TEMP_FET_GPIO_Port;
  temperature_self->pwr_gpio.pin = TEMP_FET_Pin;
  temperature_self->timer_timeout = false;

  temperature_self->self_test = _temperature_self_test;
  temperature_self->get_readings = _temperature_get_readings;
  temperature_self->start_timer = _temperature_start_timer;
  temperature_self->stop_timer = _temperature_stop_timer;
  temperature_self->on = _temperature_on;
  temperature_self->off = _temperature_off;

  __reset_struct_fields (clear_calibration_data);

  generic_i2c_register_io_functions (&temperature_self->i2c_driver, i2c_handle, i2c_mutex,
  TEMPERATURE_I2C_MUTEX_WAIT_TICKS,
                                     i2c1_init, i2c1_deinit, NULL,
                                     NULL,
                                     true);
}

void temperature_deinit ( void )
{
  temperature_self->i2c_driver->deinit ();
}

void temperature_timer_expired ( ULONG expiration_input )
{
  temperature_self->timer_timeout = true;
}

bool temperature_get_timeout_status ( void )
{
  return temperature_self->timer_timeout;
}

static temperature_return_code_t _temperature_self_test ( float *optional_reading )
{
  if ( __init_sensor () )
  {
    if ( optional_reading == NULL )
    {
      return TEMPERATURE_SUCCESS;
    }
    else
    {
      return _temperature_get_readings (true, optional_reading);
    }
  }

  return TEMPERATURE_COMMUNICATION_ERROR;
}

static temperature_return_code_t _temperature_get_readings ( bool get_single_reading,
                                                             float *temperature )
{
  temperature_return_code_t return_code = TEMPERATURE_SUCCESS;
  uint8_t command;
  uint8_t read_data[3] =
    { 0 };
  int32_t num_readings = (get_single_reading) ?
      1 : temperature_self->global_config->total_temp_samples;
  float readings_accumulator = 0;

  for ( int i = 0; i < num_readings; i++ )
  {

    command = TSYS01_ADC_TEMP_CONV;
    if ( temperature_self->i2c_driver.write (&temperature_self->i2c_driver, TSYS01_ADDR, &command,
                                             sizeof(command))
         != GENERIC_I2C_OK )
    {
      return_code = TEMPERATURE_COMMUNICATION_ERROR;
      return return_code;
    }

    tx_thread_sleep (1);

    command = TSYS01_ADC_READ;
    if ( temperature_self->i2c_driver.write (&temperature_self->i2c_driver, TSYS01_ADDR, &command,
                                             sizeof(command))
         != GENERIC_I2C_OK )
    {
      return_code = TEMPERATURE_COMMUNICATION_ERROR;
      return return_code;
    }

    if ( temperature_self->i2c_driver.read (&temperature_self->i2c_driver, TSYS01_ADDR,
                                            &(read_data[0]), sizeof(read_data))
         != GENERIC_I2C_OK )
    {
      return_code = TEMPERATURE_COMMUNICATION_ERROR;
      return return_code;
    }

    temperature_self->D1 = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

    readings_accumulator += __calculate_temp ();
  }

  temperature_self->converted_temp = readings_accumulator / num_readings;

  *temperature = temperature_self->converted_temp;

  return return_code;
}

static temperature_return_code_t _temperature_start_timer ( uint16_t timeout_in_minutes )
{
  uint16_t timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  temperature_return_code_t ret = TEMPERATURE_SUCCESS;

  if ( tx_timer_change (temperature_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = TEMPERATURE_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (temperature_self->timer) != TX_SUCCESS )
  {
    ret = TEMPERATURE_TIMER_ERROR;
  }

  return ret;
}

static temperature_return_code_t _temperature_stop_timer ( void )
{
  return (tx_timer_deactivate (temperature_self->timer) == TX_SUCCESS) ?
      TEMPERATURE_SUCCESS : TEMPERATURE_TIMER_ERROR;
}

static void _temperature_on ( void )
{
  HAL_GPIO_WritePin (temperature_self->pwr_gpio.port, temperature_self->pwr_gpio.pin, GPIO_PIN_SET);
  tx_thread_sleep (1);
}

static void _temperature_off ( void )
{
  HAL_GPIO_WritePin (temperature_self->pwr_gpio.port, temperature_self->pwr_gpio.pin,
                     GPIO_PIN_RESET);
}

static bool __init_sensor ( void )
{
  uint8_t command = TSYS01_RESET;
  uint8_t read_data[2] =
    { 0 };

  if ( temperature_self->i2c_driver.init () != I2C_OK )
  {
    return false;
  }

  // Reset the TSYS01, per datasheet
  if ( temperature_self->i2c_driver.write (&temperature_self->i2c_driver, TSYS01_ADDR, &command,
                                           sizeof(command))
       != GENERIC_I2C_OK )
  {
    return false;
  }

  tx_thread_sleep (1);
  // Read calibration values
  for ( uint8_t i = 0; i < 8; i++ )
  {
    command = TSYS01_PROM_READ + (i * 2);
    if ( temperature_self->i2c_driver.write (&temperature_self->i2c_driver, TSYS01_ADDR, &command,
                                             sizeof(command))
         != GENERIC_I2C_OK )
    {
      return false;
    }

    if ( temperature_self->i2c_driver.read (&temperature_self->i2c_driver, TSYS01_ADDR,
                                            &(read_data[0]), sizeof(read_data))
         != GENERIC_I2C_OK )
    {
      return false;
    }

    temperature_self->C[i] = (float) ((read_data[0] << 8) | read_data[1]);
  }

  return true;
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

  temperature_self->converted_temp = 0.0f;
  temperature_self->D1 = 0;
  temperature_self->adc = 0;
}

// @formatter:on

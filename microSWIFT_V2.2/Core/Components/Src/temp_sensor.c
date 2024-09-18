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

// @formatter:off
// Object instance pointer
static Temperature *self;

// Struct functions
static temperature_return_code_t _temperature_self_test ( float *optional_reading );
static temperature_return_code_t _temperature_get_readings ( bool get_single_reading, float *temperature );
static void                      _temperature_on ( void );
static void                      _temperature_off ( void );

// Helper functions
static bool                      __init_sensor ( void );
static float                     __calculate_temp ( void );
static void                      __reset_struct_fields ( bool reset_calibration );
// @formatter:on

void temperature_init ( microSWIFT_configuration *global_config, Temperature *struct_ptr,
                        I2C_HandleTypeDef *i2c_handle, TX_EVENT_FLAGS_GROUP *error_flags,
                        TX_MUTEX *i2c_mutex, bool clear_calibration_data )
{
  self = struct_ptr;

  self->global_config = global_config;
  self->i2c_handle = i2c_handle;
  self->error_flags = error_flags;
  self->i2c_mutex = i2c_mutex;
  self->pwr_gpio.port = TEMP_FET_GPIO_Port;
  self->pwr_gpio.pin = TEMP_FET_Pin;

  self->on = _temperature_on;
  self->off = _temperature_off;
  self->self_test = _temperature_self_test;
  self->get_readings = _temperature_get_readings;

  __reset_struct_fields (clear_calibration_data);
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
      1 : self->global_config->total_temp_samples;
  float readings_accumulator = 0;

  for ( int i = 0; i < num_readings; i++ )
  {

    command = TSYS01_ADC_TEMP_CONV;
    if ( HAL_I2C_Master_Transmit (self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
         != HAL_OK )
    {
      return_code = TEMPERATURE_CONVERSION_ERROR;
      return return_code;
    }

    tx_thread_sleep (1);

    command = TSYS01_ADC_READ;
    if ( HAL_I2C_Master_Transmit (self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
         != HAL_OK )
    {
      return_code = TEMPERATURE_CONVERSION_ERROR;
      return return_code;
    }

    if ( HAL_I2C_Master_Receive (self->i2c_handle, TSYS01_ADDR, &(read_data[0]), sizeof(read_data),
                                 10)
         != HAL_OK )
    {
      return_code = TEMPERATURE_CONVERSION_ERROR;
      return return_code;
    }

    self->D1 = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

    readings_accumulator += __calculate_temp ();
  }

  self->converted_temp = readings_accumulator / num_readings;

  *temperature = self->converted_temp;

  return return_code;
}

static void _temperature_on ( void )
{
  HAL_GPIO_WritePin (self->pwr_gpio.port, self->pwr_gpio.pin, GPIO_PIN_SET);
  tx_thread_sleep (1);
}

static void _temperature_off ( void )
{
  HAL_GPIO_WritePin (self->pwr_gpio.port, self->pwr_gpio.pin, GPIO_PIN_RESET);
}

static bool __init_sensor ( void )
{
  uint8_t command = TSYS01_RESET;
  uint8_t read_data[2] =
    { 0 };
  // Reset the TSYS01, per datasheet
  if ( HAL_I2C_Master_Transmit (self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
       != HAL_OK )
  {
    return false;
  }

  tx_thread_sleep (1);
  // Read calibration values
  for ( uint8_t i = 0; i < 8; i++ )
  {
    command = TSYS01_PROM_READ + (i * 2);
    if ( HAL_I2C_Master_Transmit (self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
         != HAL_OK )
    {
      return false;
    }

    if ( HAL_I2C_Master_Receive (self->i2c_handle, TSYS01_ADDR, &(read_data[0]), sizeof(read_data),
                                 10)
         != HAL_OK )
    {
      return false;
    }

    self->C[i] = (float) ((read_data[0] << 8) | read_data[1]);
  }

  return true;
}

static float __calculate_temp ( void )
{
  float temp = 0.0;
  self->adc = self->D1 / 256;
  temp = (-2.0f) * self->C[1] / 1000000000000000000000.0f * pow (self->adc, 4)
         + 4.0f * self->C[2] / 10000000000000000.0f * pow (self->adc, 3)
         + (-2.0f) * self->C[3] / 100000000000.0f * pow (self->adc, 2)
         + 1.0f * self->C[4] / 1000000.0f * self->adc + (-1.5f) * self->C[5] / 100.0f;
  return temp;
}

static void __reset_struct_fields ( bool reset_calibration )
{
  if ( reset_calibration )
  {
    memset (self->C, 0, sizeof(self->C));
  }

  self->converted_temp = 0.0f;
  self->D1 = 0;
  self->adc = 0;
}

// @formatter:on

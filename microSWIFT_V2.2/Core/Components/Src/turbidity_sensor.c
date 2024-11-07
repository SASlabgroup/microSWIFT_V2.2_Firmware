/*
 * turbidity_sensor.c
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#include "turbidity_sensor.h"

// @formatter:off
static Turbidity_Sensor *turbidity_self;

// Struct functions
static uSWIFT_return_code_t _turbidity_sensor_self_test (void);
static uSWIFT_return_code_t _turbidity_sensor_setup_sensor (void);
static uSWIFT_return_code_t _turbidity_sensor_get_sample (void);
static uSWIFT_return_code_t _turbidity_sensor_process_measurements (void);
static uSWIFT_return_code_t _turbidity_sensor_start_timer ( uint16_t timeout_in_minutes );
static uSWIFT_return_code_t _turbidity_sensor_stop_timer (void);
static void                 _turbidity_sensor_on (void);
static void                 _turbidity_sensor_off (void);
// I/O functions for the sensor
static int32_t              _turbidity_sensor_i2c_init ( void );
static int32_t              _turbidity_sensor_i2c_deinit ( void );
static int32_t              _turbidity_sensor_i2c_read ( void *unused_handle, uint16_t bus_address,
                                                          uint16_t reg_address, uint8_t *read_data,
                                                          uint16_t data_length );
static int32_t              _turbidity_sensor_i2c_write ( void *unused_handle, uint16_t bus_address,
                                                           uint16_t reg_address, uint8_t *write_data,
                                                           uint16_t data_length );
static void                 _turbidity_sensor_ms_delay ( uint32_t delay );
// Helper functions


// @formatter:on

void turbidity_sensor_init ( Turbidity_Sensor *struct_ptr, microSWIFT_configuration *global_config,
                             I2C_HandleTypeDef *i2c_handle, TX_TIMER *timer,
                             TX_SEMAPHORE *sensor_i2c_sema, uint16_t *samples_buffer )
{
  turbidity_self = struct_ptr;

  turbidity_self->global_config = global_config;

  turbidity_self->i2c_handle = i2c_handle;
  turbidity_self->i2c_sema = sensor_i2c_sema;

  turbidity_self->timer = timer;

  turbidity_self->samples_series = samples_buffer;

  turbidity_self->timer_timeout = false;

  turbidity_self->self_test = _turbidity_sensor_self_test;
  turbidity_self->setup_sensor = _turbidity_sensor_setup_sensor;
  turbidity_self->get_sample = _turbidity_sensor_get_sample;
  turbidity_self->process_measurements = _turbidity_sensor_process_measurements;
  turbidity_self->start_timer = _turbidity_sensor_start_timer;
  turbidity_self->stop_timer = _turbidity_sensor_stop_timer;
  turbidity_self->on = _turbidity_sensor_on;
  turbidity_self->off = _turbidity_sensor_off;
}

void turbidity_deinit ( void )
{
  if ( turbidity_self->dev_ctx.deinit != NULL )
  {
    turbidity_self->dev_ctx.deinit ();
  }
}

void turbidity_timer_expired ( ULONG expiration_input )
{
  turbidity_self->timer_timeout = true;
}

bool turbidity_get_timeout_status ( void )
{
  return turbidity_self->timer_timeout;
}

static uSWIFT_return_code_t _turbidity_sensor_self_test ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_setup_sensor ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_get_sample ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_process_measurements ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_start_timer ( uint16_t timeout_in_minutes )
{
  ULONG timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( tx_timer_change (turbidity_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (turbidity_self->timer) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
  }

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_stop_timer ( void )
{
  return (tx_timer_deactivate (turbidity_self->timer) == TX_SUCCESS) ?
      uSWIFT_SUCCESS : uSWIFT_TIMER_ERROR;
}

static void _turbidity_sensor_on ( void )
{
  HAL_GPIO_WritePin (TURBIDITY_FET_GPIO_Port, TURBIDITY_FET_Pin, GPIO_PIN_SET);
}

static void _turbidity_sensor_off ( void )
{
  HAL_GPIO_WritePin (TURBIDITY_FET_GPIO_Port, TURBIDITY_FET_Pin, GPIO_PIN_RESET);
}

static int32_t _turbidity_sensor_i2c_init ( void )
{
  return i2c2_init ();
}

static int32_t _turbidity_sensor_i2c_deinit ( void )
{
  return i2c2_deinit ();
}

static int32_t _turbidity_sensor_i2c_read ( void *unused_handle, uint16_t bus_address,
                                            uint16_t reg_address, uint8_t *read_data,
                                            uint16_t data_length )
{
  (void) unused_handle;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( HAL_I2C_Mem_Read_IT (turbidity_self->i2c_handle, bus_address, reg_address, 1, read_data,
                            data_length)
       != HAL_OK )
  {
    ret = uSWIFT_COMMS_ERROR;
    return ret;
  }

  if ( tx_semaphore_get (turbidity_self->i2c_sema, TURBIDITY_I2C_TIMEOUT) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMEOUT;
  }

  return ret;
}

static int32_t _turbidity_sensor_i2c_write ( void *unused_handle, uint16_t bus_address,
                                             uint16_t reg_address, uint8_t *write_data,
                                             uint16_t data_length )
{
  (void) unused_handle;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( HAL_I2C_Mem_Write_IT (turbidity_self->i2c_handle, bus_address, reg_address, 1, write_data,
                             data_length)
       != HAL_OK )
  {
    ret = uSWIFT_COMMS_ERROR;
    return ret;
  }

  if ( tx_semaphore_get (turbidity_self->i2c_sema, TURBIDITY_I2C_TIMEOUT) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMEOUT;
  }

  return ret;
}

static void _turbidity_sensor_ms_delay ( uint32_t delay )
{
  if ( delay == 0 )
  {
    tx_thread_relinquish ();
  }
  else
  {
    tx_thread_sleep (delay);
  }
}

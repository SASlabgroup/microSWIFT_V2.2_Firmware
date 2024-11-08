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
static uSWIFT_return_code_t _turbidity_sensor_take_measurement (void);
static uSWIFT_return_code_t _turbidity_sensor_get_raw_counts (uint16_t *raw_counts);
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
                             TX_SEMAPHORE *sensor_i2c_sema, int32_t *samples_buffer )
{
  turbidity_self = struct_ptr;

  turbidity_self->global_config = global_config;

  turbidity_self->i2c_handle = i2c_handle;
  turbidity_self->i2c_sema = sensor_i2c_sema;

  turbidity_self->timer = timer;

  turbidity_self->samples_series = samples_buffer;
  memset (&(turbidity_self->averages_series[0]), 0, sizeof(turbidity_self->averages_series));
  turbidity_self->samples_counter = 0;
  turbidity_self->raw_count = 0;

  turbidity_self->timer_timeout = false;

  turbidity_self->self_test = _turbidity_sensor_self_test;
  turbidity_self->setup_sensor = _turbidity_sensor_setup_sensor;
  turbidity_self->take_measurement = _turbidity_sensor_take_measurement;
  turbidity_self->get_raw_counts = _turbidity_sensor_get_raw_counts;
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
  uint8_t id = 0;

  if ( vcnl4010_register_io_functions (&turbidity_self->dev_ctx, _turbidity_sensor_i2c_init,
                                       _turbidity_sensor_i2c_deinit, _turbidity_sensor_i2c_write,
                                       _turbidity_sensor_i2c_read, _turbidity_sensor_ms_delay)
       != uSWIFT_SUCCESS )
  {
    return uSWIFT_INITIALIZATION_ERROR;
  }

  ret = vcnl4010_get_id (&turbidity_self->dev_ctx, &id);
  if ( (ret != uSWIFT_SUCCESS) || (id != PROD_ID_REV_REG_RESET_VAL) )
  {
    return uSWIFT_INITIALIZATION_ERROR;
  }

  ret = turbidity_self->setup_sensor ();
  if ( ret != uSWIFT_SUCCESS )
  {
    return uSWIFT_INITIALIZATION_ERROR;
  }

  ret = turbidity_self->take_measurement ();

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_setup_sensor ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  ret |= vcnl4010_auto_offset_comp_config (&turbidity_self->dev_ctx, true);

  ret |= vcnl4010_set_led_current (&turbidity_self->dev_ctx, _50_MA);

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_take_measurement ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  int32_t result = 0;
  uint16_t intermediate_result = 0;

  // Take 4 measurements --> prox (+), ambient (-), ambient (-), prox (+)
  for ( int i = 0; i < 4; i++ )
  {
    switch ( i )
    {
      case 0:
        if ( vcnl4010_get_proximity_reading (&turbidity_self->dev_ctx, &intermediate_result)
             != uSWIFT_SUCCESS )
        {
          return uSWIFT_COMMS_ERROR;
        }

        result += intermediate_result;

        break;

      case 1 ... 2:
        if ( vcnl4010_get_ambient_reading (&turbidity_self->dev_ctx, &intermediate_result)
             != uSWIFT_SUCCESS )
        {
          return uSWIFT_COMMS_ERROR;
        }

        result -= intermediate_result;

        break;

      case 3:
        if ( vcnl4010_get_proximity_reading (&turbidity_self->dev_ctx, &intermediate_result)
             != uSWIFT_SUCCESS )
        {
          return uSWIFT_COMMS_ERROR;
        }

        result += intermediate_result;

        break;

      default:
        break;

    }
  }

  turbidity_self->samples_series[turbidity_self->samples_counter] = result;

  turbidity_self->raw_count = result;

  if ( ++turbidity_self->samples_counter == turbidity_self->global_config->total_turbidity_samples )
  {
    ret = uSWIFT_DONE_SAMPLING;
  }

  if ( turbidity_self->samples_counter % 60 == 0 )
  {
    turbidity_self->process_measurements ();
  }

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_get_raw_counts ( uint16_t *raw_counts )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( turbidity_self->samples_counter == 0 )
  {
    raw_counts = NULL;
    return uSWIFT_NO_SAMPLES_ERROR;
  }

  *raw_counts = turbidity_self->raw_count;

  return ret;
}

static uSWIFT_return_code_t _turbidity_sensor_process_measurements ( void )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  static uint32_t averages_index = 0;
  uint64_t temp_accumulator = 0;
  uint32_t sample_index = turbidity_self->samples_counter;

  for ( int i = 0; i < 60; i++, sample_index-- )
  {
    temp_accumulator += turbidity_self->samples_series[sample_index];
  }

  temp_accumulator /= 60;

  turbidity_self->averages_series[averages_index] = (int32_t) temp_accumulator;

  averages_index++;

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

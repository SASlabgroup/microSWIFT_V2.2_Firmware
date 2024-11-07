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
static int32_t              _turbidity_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                          uint16_t reg_address, uint8_t *read_data,
                                                          uint16_t data_length );
static int32_t              _turbidity_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
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

}

void turbidity_deinit ( void )
{
  if ( turbidity_self->dev_ctx.deinit != NULL )
  {
    turbidity_self->dev_ctx.deinit ();
  }
}

void turbidity_timer_expired ( ULONG expiration_input );
bool turbidity_get_timeout_status ( void );

static uSWIFT_return_code_t _turbidity_sensor_self_test ( void );
static uSWIFT_return_code_t _turbidity_sensor_setup_sensor ( void );
static uSWIFT_return_code_t _turbidity_sensor_get_sample ( void );
static uSWIFT_return_code_t _turbidity_sensor_process_measurements ( void );
static uSWIFT_return_code_t _turbidity_sensor_start_timer ( uint16_t timeout_in_minutes );
static uSWIFT_return_code_t _turbidity_sensor_stop_timer ( void );
static void _turbidity_sensor_on ( void );
static void _turbidity_sensor_off ( void );

static int32_t _turbidity_sensor_i2c_init ( void );
static int32_t _turbidity_sensor_i2c_deinit ( void );
static int32_t _turbidity_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                     uint16_t reg_address, uint8_t *read_data,
                                                     uint16_t data_length );
static int32_t _turbidity_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
                                                      uint16_t reg_address, uint8_t *write_data,
                                                      uint16_t data_length );
static void _turbidity_sensor_ms_delay ( uint32_t delay );

/*
 * light_sensor.c
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#include "light_sensor.h"
#include "gpio.h"
#include "tx_api.h"
#include "i2c.h"

// @formatter:off
static Light_Sensor *light_self;
static as7341_gpio_int_struct gpio_struct;

// Struct functions
static light_return_code_t  _light_sensor_self_test (uint16_t *clear_channel_reading);
static light_return_code_t  _light_sensor_setup_sensor (void);
static light_return_code_t  _light_sensor_read_all_channels (void);
static light_return_code_t  _light_sensor_start_timer ( uint16_t timeout_in_minutes );
static light_return_code_t  _light_sensor_stop_timer ( void );
static void                 _light_sensor_get_measurements (uint16_t *buffer);
static void                 _light_sensor_get_single_measurement (uint16_t *measurement, light_channel_index_t which_channel);
static void                 _light_sensor_on (void);
static void                 _light_sensor_off (void);

// Functions neccessary for the AS7341 pins
static bool                 __as7341_wait_on_int (uint32_t timeout_ms);
static GPIO_PinState        __get_as7341_int_pin_state ( void );
static GPIO_PinState        __get_as7341_gpio_pin_state ( void );
static void                 __set_as7341_int_pin_state ( GPIO_PinState state );
static void                 __set_as7341_gpio_pin_state ( GPIO_PinState state );
// I/O functions for the sensor
static int32_t              _light_sensor_i2c_init ( void );
static int32_t              _light_sensor_i2c_deinit ( void );
static int32_t              _light_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                          uint16_t reg_address, uint8_t *read_data,
                                                          uint16_t data_length );
static int32_t              _light_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
                                                           uint16_t reg_address, uint8_t *write_data,
                                                           uint16_t data_length );
static void                 _light_sensor_ms_delay ( uint32_t delay );

// @formatter:on
void light_sensor_init ( Light_Sensor *struct_ptr, I2C_HandleTypeDef *i2c_handle,
                         TX_SEMAPHORE *int_pin_sema )
{
  light_self = struct_ptr;
  light_self->gpio_handle = &gpio_struct;

  light_self->i2c_handle = i2c_handle;
  light_self->int_pin_sema = int_pin_sema;

  light_self->smux_assignment_low_channels.adc_assignments[0] = F1;
  light_self->smux_assignment_low_channels.adc_assignments[1] = F2;
  light_self->smux_assignment_low_channels.adc_assignments[2] = F3;
  light_self->smux_assignment_low_channels.adc_assignments[3] = F4;
  light_self->smux_assignment_low_channels.adc_assignments[4] = F5;
  light_self->smux_assignment_low_channels.adc_assignments[5] = F6;

  light_self->smux_assignment_high_channels.adc_assignments[0] = F7;
  light_self->smux_assignment_high_channels.adc_assignments[1] = F8;
  light_self->smux_assignment_high_channels.adc_assignments[2] = NIR;
  light_self->smux_assignment_high_channels.adc_assignments[3] = CLEAR;
  light_self->smux_assignment_high_channels.adc_assignments[4] = DARK;
  light_self->smux_assignment_high_channels.adc_assignments[5] = ADC_DISABLE;

  light_self->gpio_handle->int_pin.port = AS7341_INT_GPIO_Port;
  light_self->gpio_handle->int_pin.pin = AS7341_INT_Pin;
  light_self->gpio_handle->gpio_pin.port = AS7341_GPIO_GPIO_Port;
  light_self->gpio_handle->gpio_pin.pin = AS7341_GPIO_Pin;
  light_self->gpio_handle->wait_on_int = __as7341_wait_on_int;
  light_self->gpio_handle->get_int_pin_state = __get_as7341_int_pin_state;
  light_self->gpio_handle->get_gpio_pin_state = __get_as7341_gpio_pin_state;
  light_self->gpio_handle->set_int_pin_state = __set_as7341_int_pin_state;
  light_self->gpio_handle->set_gpio_pin_state = __set_as7341_gpio_pin_state;

  light_self->fet.port = LIGHT_FET_GPIO_Port;
  light_self->fet.pin = LIGHT_FET_Pin;

  memset (&(light_self->channel_data[0]), 0, sizeof(light_self->channel_data));

  light_self->current_bank = REG_BANK_UNKNOWN;

  light_self->sensor_gain = GAIN_64X;

  light_self->timer_timeout = false;

  light_self->self_test = _light_sensor_self_test;
  light_self->setup_sensor = _light_sensor_setup_sensor;
  light_self->read_all_channels = _light_sensor_read_all_channels;
  light_self->get_measurements = _light_sensor_get_measurements;
  light_self->get_single_measurement = _light_sensor_get_single_measurement;
  light_self->on = _light_sensor_on;
  light_self->off = _light_sensor_off;
}

void light_deinit ( void )
{
  if ( light_self->dev_ctx.deinit != NULL )
  {
    light_self->dev_ctx.deinit ();
  }
}

void light_timer_expired ( ULONG expiration_input )
{
  light_self->timer_timeout = true;
}

bool light_get_timeout_status ( void )
{
  return light_self->timer_timeout;
}

static light_return_code_t _light_sensor_self_test ( uint16_t *clear_channel_reading )
{
  light_return_code_t ret = LIGHT_SUCCESS;
  ;
  uint8_t id;

  // Initialize the I/O interface
  if ( as7341_register_io_functions (&light_self->dev_ctx, _light_sensor_i2c_init,
                                     _light_sensor_i2c_deinit, _light_sensor_i2c_write_blocking,
                                     _light_sensor_i2c_read_blocking, _light_sensor_ms_delay,
                                     light_self->gpio_handle)
       != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Get the register bank to a known state
  if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }
  else
  {
    light_self->current_bank = REG_BANK_80_PLUS;
  }

  // Check the chip ID
  if ( as7341_get_id (&light_self->dev_ctx, &id) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  if ( id != AS7341_ID )
  {
    return LIGHT_I2C_ERROR;
  }

  // Read all the channels
  ret |= light_self->read_all_channels ();
  light_self->get_single_measurement (clear_channel_reading, CLEAR_CHANNEL);

  return ret;
}

static light_return_code_t _light_sensor_setup_sensor ( void );

static light_return_code_t _light_sensor_read_all_channels ( void )
{
  UINT tx_ret;
  int32_t as7341_ret;

  // Power sensor on
  as7341_ret = as7341_power (&light_self->dev_ctx, true);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Set SYNS mode
  as7341_ret = as7341_set_integration_mode (&light_self->dev_ctx, SYNS_MODE);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Start by setting the SMUX to the lower channels
  as7341_ret = as7341_config_smux (&light_self->dev_ctx, &light_self->smux_assignment_low_channels);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Enable spectral measurement
  as7341_ret = as7341_spectral_meas_config (&light_self->dev_ctx, true);

  // Pulse the GPIO pin to kick off sampling in SYNS mode
  light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_SET);
  tx_thread_relinquish ();
  light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_RESET);

  // Wait until we get the interrupt telling us data is ready
  tx_ret = tx_semaphore_get (light_self->int_pin_sema,
                             ((INTEGRATION_TIME_MS / TX_TIMER_TICKS_PER_SECOND) + 1));
  if ( tx_ret != TX_SUCCESS )
  {
    return LIGHT_TIMEOUT;
  }

  // Read out the data
  as7341_ret = as7341_get_all_channel_data (
      &light_self->dev_ctx, (as7341_all_channel_data_struct*) &light_self->channel_data[0]);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Switch SMUX to upper channels
  as7341_ret = as7341_config_smux (&light_self->dev_ctx,
                                   &light_self->smux_assignment_high_channels);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Enable spectral measurement
  as7341_ret = as7341_spectral_meas_config (&light_self->dev_ctx, true);

  // Pulse the GPIO pin to kick off sampling in SYNS mode
  light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_SET);
  tx_thread_relinquish ();
  light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_RESET);

  // Wait until we get the interrupt telling us data is ready
  tx_ret = tx_semaphore_get (light_self->int_pin_sema,
                             ((INTEGRATION_TIME_MS / TX_TIMER_TICKS_PER_SECOND) + 1));
  if ( tx_ret != TX_SUCCESS )
  {
    return LIGHT_TIMEOUT;
  }

  // Read out the data
  as7341_ret = as7341_get_all_channel_data (
      &light_self->dev_ctx, (as7341_all_channel_data_struct*) &light_self->channel_data[6]);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Power sensor off
  as7341_ret = as7341_power (&light_self->dev_ctx, true);
  if ( as7341_ret != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  return LIGHT_SUCCESS;
}

static light_return_code_t _light_sensor_start_timer ( uint16_t timeout_in_minutes )
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

static light_return_code_t _light_sensor_stop_timer ( void )
{

}

static void _light_sensor_get_measurements ( uint16_t *buffer )
{
  memcpy (buffer, &(light_self->channel_data[0]), sizeof(light_self->channel_data));
}

static void _light_sensor_get_single_measurement ( uint16_t *measurement,
                                                   light_channel_index_t which_channel )
{
  if ( (which_channel < 0) || (which_channel > DARK_CHANNEL) )
  {
    *measurement = 0;
    return;
  }

  *measurement = light_self->channel_data[which_channel];
}

static void _light_sensor_on ( void )
{
  HAL_GPIO_WritePin (light_self->fet.port, light_self->fet.pin, GPIO_PIN_SET);
}

static void _light_sensor_off ( void )
{
  HAL_GPIO_WritePin (light_self->fet.port, light_self->fet.pin, GPIO_PIN_RESET);
}

static bool __as7341_wait_on_int ( uint32_t timeout_ms )
{
  ULONG timeout = (timeout_ms / TX_TIMER_TICKS_PER_SECOND) + 1;

  if ( tx_semaphore_get (light_self->int_pin_sema, timeout) != TX_SUCCESS )
  {
    return false;
  }

  return true;
}

static GPIO_PinState __get_as7341_int_pin_state ( void )
{
  return HAL_GPIO_ReadPin (light_self->gpio_handle->int_pin.port,
                           light_self->gpio_handle->int_pin.pin);
}

static GPIO_PinState __get_as7341_gpio_pin_state ( void )
{
  return HAL_GPIO_ReadPin (light_self->gpio_handle->gpio_pin.port,
                           light_self->gpio_handle->gpio_pin.pin);
}

void __set_as7341_int_pin_state ( GPIO_PinState state )
{
  HAL_GPIO_WritePin (light_self->gpio_handle->int_pin.port, light_self->gpio_handle->int_pin.pin,
                     state);
}

void __set_as7341_gpio_pin_state ( GPIO_PinState state )
{
  HAL_GPIO_WritePin (light_self->gpio_handle->gpio_pin.port, light_self->gpio_handle->gpio_pin.pin,
                     state);
}

static int32_t _light_sensor_i2c_init ( void )
{
  return i2c1_init ();
}

static int32_t _light_sensor_i2c_deinit ( void )
{
  return i2c1_deinit ();
}

static int32_t _light_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                 uint16_t reg_address, uint8_t *read_data,
                                                 uint16_t data_length )
{
  (void) unused_handle;
  int32_t ret = AS7341_OK;
  uint8_t read_buf[LIGHT_I2C_BUF_SIZE + 1] =
    { 0 };

  if ( !(data_length <= sizeof(read_buf)) )
  {
    return LIGHT_PARAMETERS_INVALID;
  }

// Why could they possibly need to put a bit in there for different register banks? This
// could so easily be handled in the asic
  if ( (reg_address < 0x80)
       && (reg_address != CFG0_REG_ADDR)
       && ((light_self->current_bank == REG_BANK_80_PLUS)
           || (light_self->current_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_60_74) != AS7341_OK )
    {
      light_self->current_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }
  }
  else if ( (reg_address >= 0x80)
            && (reg_address != CFG0_REG_ADDR)
            && ((light_self->current_bank == REG_BANK_60_74)
                || (light_self->current_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS) != AS7341_OK )
    {
      light_self->current_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }
  }

  read_buf[0] = (uint8_t) reg_address;
  memcpy (&(read_buf[1]), read_data, data_length);

  if ( HAL_I2C_Master_Transmit (light_self->i2c_handle, bus_address, &(read_buf[0]),
                                data_length + 1,
                                LIGHT_I2C_TIMEOUT)
       != HAL_OK )
  {
    ret = AS7341_ERROR;
  }

  memcpy (read_data, &read_buf[1], data_length);

  return ret;
}

static int32_t _light_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
                                                  uint16_t reg_address, uint8_t *write_data,
                                                  uint16_t data_length )
{
  (void) unused_handle;
  int32_t ret = AS7341_OK;
  uint8_t write_buf[LIGHT_I2C_BUF_SIZE + 1] =
    { 0 };

  if ( !(data_length <= sizeof(write_buf)) )
  {
    return LIGHT_PARAMETERS_INVALID;
  }

// Why could they possibly need to put a bit in there for different register banks? This
// could so easily be handled in the asic
  if ( (reg_address < 0x80)
       && (reg_address != CFG0_REG_ADDR)
       && ((light_self->current_bank == REG_BANK_80_PLUS)
           || (light_self->current_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_60_74) != AS7341_OK )
    {
      light_self->current_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }
  }
  else if ( (reg_address >= 0x80)
            && (reg_address != CFG0_REG_ADDR)
            && ((light_self->current_bank == REG_BANK_60_74)
                || (light_self->current_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS) != AS7341_OK )
    {
      light_self->current_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }
  }

  write_buf[0] = (uint8_t) reg_address;
  memcpy (&(write_buf[1]), write_data, data_length);

  if ( HAL_I2C_Master_Transmit (light_self->i2c_handle, bus_address, &(write_buf[0]),
                                data_length + 1,
                                LIGHT_I2C_TIMEOUT)
       != HAL_OK )
  {
    ret = AS7341_ERROR;
  }

  return ret;
}

static void _light_sensor_ms_delay ( uint32_t delay )

{
  UINT delay_ticks = (delay == 0) ?
      0 : delay / TX_TIMER_TICKS_PER_SECOND;

  if ( delay == 0 )
  {
    tx_thread_relinquish ();
  }
  else
  {
    tx_thread_sleep (delay_ticks);
  }
}

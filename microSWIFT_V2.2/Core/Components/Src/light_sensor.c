/*
 * light_sensor.c
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#include "light_sensor.h"
#include "gpio.h"
#include "tx_api.h"

// @formatter:off
static Light_Sensor *light_self;

// Functions neccessary for the AS7341 pins
static bool             __as7341_wait_on_int (uint32_t timeout_ms);
static GPIO_PinState    __get_as7341_int_pin_state ( void );
static GPIO_PinState    __get_as7341_gpio_pin_state ( void );
static void             __set_as7341_int_pin_state ( GPIO_PinState state );
static void             __set_as7341_gpio_pin_state ( GPIO_PinState state );

// @formatter:on
light_return_code_t light_sensor_init ( Light_Sensor *struct_ptr, I2C_HandleTypeDef *i2c_handle,
                                        TX_SEMAPHORE *int_pin_sema )
{
  light_return_code_t ret = LIGHT_SUCCESS;
  uint8_t id;

  light_self = struct_ptr;

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

  memset (&(light_self->channel_data[0]), 0, sizeof(light_self->channel_data));

  ret |= as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS);

  if ( ret == AS7341_OK )
  {
    light_self->current_bank = REG_BANK_80_PLUS;
  }
  else
  {
    light_self->current_bank = REG_BANK_UNKNOWN;
  }

  ret |= as7341_get_id (&light_self->dev_ctx, &id);

if (id != AS7341_ID)
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

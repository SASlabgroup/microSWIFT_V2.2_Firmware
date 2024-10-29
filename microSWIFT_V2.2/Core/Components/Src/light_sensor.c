/*
 * light_sensor.c
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#include "light_sensor.h"

static Light_Sensor *light_self;

light_return_code_t light_sensor_init ( Light_Sensor *struct_ptr, I2C_HandleTypeDef *i2c_handle,
                                        TX_SEMAPHORE *int_pin_sema )
{
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
}

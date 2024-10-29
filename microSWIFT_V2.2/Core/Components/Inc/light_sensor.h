/*
 * light_sensor.h
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LIGHT_SENSOR_H_
#define COMPONENTS_INC_LIGHT_SENSOR_H_

#include "as7341_reg.h"

// @formatter:off
typedef struct
{
  I2C_HandleTypeDef         *i2c_handle;
  as7341_smux_assignment    smux_assignment_low_channels;
  as7341_smux_assignment    smux_assignment_high_channels;
  as7341_gpio_handle        gpio_handle;
} Light_Sensor;

// @formatter:on
#endif /* COMPONENTS_INC_LIGHT_SENSOR_H_ */

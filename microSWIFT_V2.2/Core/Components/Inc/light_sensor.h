/*
 * light_sensor.h
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LIGHT_SENSOR_H_
#define COMPONENTS_INC_LIGHT_SENSOR_H_

#include "as7341_reg.h"
#include "tx_api.h"
#include "i2c.h"

#define LIGHT_I2C_BUF_SIZE 32
#define LIGHT_I2C_TIMEOUT 25
#define NUM_LIGHT_CHANNELS 11

// @formatter:off
typedef enum
{
  LIGHT_SUCCESS             = 0,
  LIGHT_I2C_ERROR           = -1,
  LIGHT_PARAMETERS_INVALID  = -2,
  LIGHT_TIMEOUT             = -3
} light_return_code_t;

typedef enum
{
  F1_CHANNEL            = 0,
  F2_CHANNEL            = 1,
  F3_CHANNEL            = 2,
  F4_CHANNEL            = 3,
  F5_CHANNEL            = 4,
  F6_CHANNEL            = 5,
  F7_CHANNEL            = 6,
  F8_CHANNEL            = 7,
  NIR_CHANNEL           = 8,
  CLEAR_CHANNEL         = 9,
  DARK_CHANNEL          = 10
} light_channel_index_t;

typedef struct
{
  I2C_HandleTypeDef         *i2c_handle;

  TX_SEMAPHORE              *int_pin_sema;

  dev_ctx_t                 dev_ctx;

  as7341_smux_assignment    smux_assignment_low_channels;
  as7341_smux_assignment    smux_assignment_high_channels;

  as7341_gpio_handle        gpio_handle;

  uint16_t                  channel_data[NUM_LIGHT_CHANNELS];

  as7341_reg_bank_t         current_bank;

  as7341_again_t            sensor_gain;

  light_return_code_t       (*self_test) (uint16_t *clear_channel_reading);
  light_return_code_t       (*setup_sensor) (void);
  light_return_code_t       (*read_all_channels) (void);
  light_return_code_t       (*get_measurements) (uint16_t *buffer);
  light_return_code_t       (*get_single_measurement) (uint16_t *measurement, light_channel_index_t which_channel);
  light_return_code_t       (*on) (void);
  light_return_code_t       (*off) (void);

} Light_Sensor;

void light_sensor_init ( Light_Sensor *struct_ptr, I2C_HandleTypeDef *i2c_handle,
                          TX_SEMAPHORE *int_pin_sema );

// @formatter:on
#endif /* COMPONENTS_INC_LIGHT_SENSOR_H_ */

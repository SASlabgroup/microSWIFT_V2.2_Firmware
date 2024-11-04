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
#include "configuration.h"

#define LIGHT_I2C_BUF_SIZE 32
#define LIGHT_I2C_TIMEOUT 5
#define NUM_LIGHT_CHANNELS 11
#define INTEGRATION_TIME_MS 50
#define LIGHT_SENSOR_BYTE_POOL_BUFFER_SIZE 86400// max size = 30 mins of qty 12 4-byte measurements every second

// @formatter:off
typedef enum
{
  LIGHT_SUCCESS             = 0,
  LIGHT_I2C_ERROR           = -1,
  LIGHT_PARAMETERS_INVALID  = -2,
  LIGHT_TIMEOUT             = -3,
  LIGHT_TIMER_ERROR         = -4,
  LIGHT_DONE_SAMPLING       = -5
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
  uint16_t f1_chan;
  uint16_t f2_chan;
  uint16_t f3_chan;
  uint16_t f4_chan;
  uint16_t f5_chan;
  uint16_t f6_chan;
  uint16_t f7_chan;
  uint16_t f8_chan;
  uint16_t nir_chan;
  uint16_t clear_chan;
  uint16_t dark_chan;
  uint16_t spare_chan;
} light_raw_counts;

typedef struct
{
  uint32_t f1_chan;
  uint32_t f2_chan;
  uint32_t f3_chan;
  uint32_t f4_chan;
  uint32_t f5_chan;
  uint32_t f6_chan;
  uint32_t f7_chan;
  uint32_t f8_chan;
  uint32_t nir_chan;
  uint32_t clear_chan;
  uint32_t dark_chan;
  uint32_t spare_chan;
} light_basic_counts;

typedef struct
{
  microSWIFT_configuration  *global_config;

  I2C_HandleTypeDef         *i2c_handle;

  TX_SEMAPHORE              *int_pin_sema;
  TX_SEMAPHORE              *i2c_sema;

  TX_TIMER                  *timer;

  dev_ctx_t                 dev_ctx;

  as7341_smux_assignment    smux_assignment_low_channels;
  as7341_smux_assignment    smux_assignment_high_channels;

  as7341_gpio_handle        gpio_handle;

  gpio_pin_struct           fet;

  light_raw_counts          raw_counts;

  light_basic_counts        basic_counts;

  int32_t                   as7341_current_reg_bank;

  as7341_again_t            sensor_gain;

  bool                      timer_timeout;

  uint32_t                  total_samples;
  light_basic_counts        samples_max;
  light_basic_counts        samples_min;
  light_basic_counts        samples_averages_accumulator;
  light_basic_counts        *samples_series;


  light_return_code_t       (*self_test) (void);
  light_return_code_t       (*setup_sensor) (void);
  light_return_code_t       (*read_all_channels) (void);
  light_return_code_t       (*start_timer) ( uint16_t timeout_in_minutes );
  light_return_code_t       (*stop_timer) ( void );
  light_return_code_t       (*process_measurements) (void);
  light_return_code_t       (*get_samples_averages) (void);
  void                      (*get_raw_measurements) (light_raw_counts *buffer);
  void                      (*get_basic_counts) (light_basic_counts *buffer);
  void                      (*get_single_measurement) (uint16_t *raw_measurement, uint32_t *basic_count, light_channel_index_t which_channel);
  void                      (*on) (void);
  void                      (*off) (void);

} Light_Sensor;

void light_sensor_init ( Light_Sensor *struct_ptr, microSWIFT_configuration *global_config,
                         light_basic_counts *samples_series_buffer, I2C_HandleTypeDef *i2c_handle,
                         TX_TIMER *timer, TX_SEMAPHORE *int_pin_sema,
                         TX_SEMAPHORE *light_sensor_i2c_sema );
void light_deinit ( void );
void light_timer_expired ( ULONG expiration_input );
bool light_get_timeout_status ( void );

// @formatter:on
#endif /* COMPONENTS_INC_LIGHT_SENSOR_H_ */

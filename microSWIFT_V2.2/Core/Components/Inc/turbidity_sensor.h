/*
 * turbidity_sensor.h
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_TURBIDITY_SENSOR_H_
#define COMPONENTS_INC_TURBIDITY_SENSOR_H_

#include "vcnl4010_reg.h"
#include "microSWIFT_return_codes.h"
#include "configuration.h"
#include "i2c.h"
#include "tx_api.h"
#include "gpio.h"

// @formatter:off

typedef struct
{
  microSWIFT_configuration  *global_config;

  I2C_HandleTypeDef         *i2c_handle;

  TX_SEMAPHORE              *i2c_sema;

  TX_TIMER                  *timer;

  uint16_t                  *samples_series;

  dev_ctx_t                 dev_ctx;

  bool                      timer_timeout;

  uSWIFT_return_code_t      (*self_test) (void);
  uSWIFT_return_code_t      (*setup_sensor) (void);
  uSWIFT_return_code_t      (*get_sample) (void);
  uSWIFT_return_code_t      (*process_measurements) (void);
  uSWIFT_return_code_t      (*start_timer) ( uint16_t timeout_in_minutes );
  uSWIFT_return_code_t      (*stop_timer) (void);
  void                      (*on) (void);
  void                      (*off) (void);
} Turbidity_Sensor;

void turbidity_sensor_init          ( Turbidity_Sensor *struct_ptr, microSWIFT_configuration *global_config,
                                      I2C_HandleTypeDef *i2c_handle, TX_TIMER *timer,
                                      TX_SEMAPHORE *sensor_i2c_sema, uint16_t *samples_buffer );
void turbidity_deinit               ( void );
void turbidity_timer_expired        ( ULONG expiration_input );
bool turbidity_get_timeout_status   ( void );
// @formatter:on
#endif /* COMPONENTS_INC_TURBIDITY_SENSOR_H_ */

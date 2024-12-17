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
#include "sbd.h"
#include "i2c.h"
#include "tx_api.h"
#include "gpio.h"

// @formatter:off

#define TURBIDITY_I2C_TIMEOUT 50
#define TURBIDITY_SENSOR_SAMPLE_BUFFER_SIZE 14400

typedef struct
{
  microSWIFT_configuration  *global_config;

  TX_TIMER                  *timer;

  uint16_t                  *ambient_series;
  uint16_t                  *proximity_series;

  uint16_t                  ambient_averages_series[60]; // more than we would ever need
  uint16_t                  proximity_averages_series[60]; // more than we would ever need
  int32_t                   start_lat;
  int32_t                   start_lon;
  int32_t                   end_lat;
  int32_t                   end_lon;
  uint32_t                  start_timestamp;
  uint32_t                  end_timestamp;

  uint32_t                  samples_counter;

  dev_ctx_t                 dev_ctx;

  bool                      timer_timeout;

  uSWIFT_return_code_t      (*self_test) (void);
  uSWIFT_return_code_t      (*setup_sensor) (void);
  uSWIFT_return_code_t      (*take_measurement) (void);
  uSWIFT_return_code_t      (*get_most_recent_measurement) (uint16_t *ambient, uint16_t *proximity);
  uSWIFT_return_code_t      (*process_measurements) (void);
  uSWIFT_return_code_t      (*start_timer) (uint16_t timeout_in_minutes);
  uSWIFT_return_code_t      (*stop_timer) (void);
  void                      (*assemble_telemetry_message_element) (sbd_message_type_53_element *msg);
  void                      (*standby) (void);
  void                      (*idle) (void);
} Turbidity_Sensor;

void turbidity_sensor_init          ( Turbidity_Sensor *struct_ptr, microSWIFT_configuration *global_config,
                                      TX_TIMER *timer, uint16_t *ambient_buffer, uint16_t *proximity_buffer );
void turbidity_timer_expired        ( ULONG expiration_input );
bool turbidity_get_timeout_status   ( void );
void turbidity_reset_sample_counter ( void );
// @formatter:on
#endif /* COMPONENTS_INC_TURBIDITY_SENSOR_H_ */

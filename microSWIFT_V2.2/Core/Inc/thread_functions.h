/*
 * thread_functions.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef INC_THREAD_FUNCTIONS_H_
#define INC_THREAD_FUNCTIONS_H_

#include "stdbool.h"
#include "tx_api.h"
#include "main.h"
#include "gnss.h"
#include "ct_sensor.h"
#include "iridium.h"
#include "temp_sensor.h"
#include "turbidity_sensor.h"
#include "light_sensor.h"
#include "accelerometer_sensor.h"
#include "iridium.h"

#define ALL_EVENT_FLAGS (0xFFFFFFFF)

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE = 1,
  TEST_PASSED_LED_SEQUENCE = 2,
  TEST_FAILED_LED_SEQUENCE = 3
} led_sequence_t;

typedef enum
{
  RTC_SERVER_THREAD = 0, // always active
  UART_LOGGER_THREAD = 1, // always active
  CONTROL_THREAD = 2, // always active
  GNSS_THREAD = 3,
  CT_THREAD = 4,
  TEMPERATURE_THREAD = 5,
  TURBIDITY_THREAD = 6,
  LIGHT_THREAD = 7,
  ACCELEROMETER_THREAD = 8,
  NED_WAVES_THREAD = 9,
  IRIDIUM_THREAD = 10
} thread_t;

bool startup_procedure ( void );

bool gnss_apply_config ( GNSS *gnss );
bool ct_self_test ( CT *ct, ct_samples *self_test_readings );
bool temperature_self_test ( Temperature *temperature, float *self_test_temp );
bool turbidity_self_test ( void );
bool light_self_test ( void );
bool accelerometer_self_test ( void );
bool iridium_apply_config ( Iridium *iridium );

bool is_first_sample_window ( void );

ULONG get_current_flags ( TX_EVENT_FLAGS_GROUP *event_flags );
void clear_event_flags ( TX_EVENT_FLAGS_GROUP *event_flags );

void led_sequence ( led_sequence_t sequence );
void jump_to_end_of_window ( ULONG error_bits_to_set );
void jump_to_waves ( void );
void send_error_message ( ULONG error_flags );
void shut_it_all_down ( void );

#endif /* INC_THREAD_FUNCTIONS_H_ */

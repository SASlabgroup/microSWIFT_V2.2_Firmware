/*
 * thread_functions.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef INC_THREADX_SUPPORT_H_
#define INC_THREADX_SUPPORT_H_

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

// @formatter:off

#define ALL_EVENT_FLAGS (0xFFFFFFFF)
#define STARTUP_SEQUENCE_MAX_WAIT_TICKS (TX_TIMER_TICKS_PER_SECOND * 20)

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE      = 1,
  TEST_PASSED_LED_SEQUENCE  = 2,
  TEST_FAILED_LED_SEQUENCE  = 3
} led_sequence_t;

bool        startup_procedure (  microSWIFT_configuration *global_config );

bool        gnss_apply_config ( GNSS *gnss );
bool        ct_self_test ( CT *ct, ct_samples *self_test_readings );
bool        temperature_self_test ( Temperature *temperature, float *self_test_temp );
bool        turbidity_self_test ( void );
bool        light_self_test ( void );
bool        accelerometer_self_test ( void );
bool        iridium_apply_config ( Iridium *iridium );

void        expansion_timer_1_expired(ULONG expiration_input);
void        expansion_timer_2_expired(ULONG expiration_input);
void        expansion_timer_3_expired(ULONG expiration_input);

bool        is_first_sample_window ( void );

ULONG       get_current_flags ( TX_EVENT_FLAGS_GROUP *event_flags );
void        clear_event_flags ( TX_EVENT_FLAGS_GROUP *event_flags );

uint32_t    ticks_from_milliseconds ( uint32_t milliseconds );

void        led_sequence ( led_sequence_t sequence );
void        jump_to_end_of_window ( ULONG error_bits_to_set );
void        jump_to_waves ( void );
void        send_error_message ( Iridium *iridium, ULONG error_flags );
void        shut_it_all_down ( void );

// @formatter:on
#endif /* INC_THREADX_SUPPORT_H_ */

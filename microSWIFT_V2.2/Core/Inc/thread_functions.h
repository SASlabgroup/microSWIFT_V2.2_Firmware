/*
 * thread_functions.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef INC_THREAD_FUNCTIONS_H_
#define INC_THREAD_FUNCTIONS_H_

#include "stdbool.h"

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE = 1,
  TEST_PASSED_LED_SEQUENCE = 2,
  TEST_NON_CRITICAL_FAULT_LED_SEQUENCE = 3,
  TEST_CRITICAL_FAULT_LED_SEQUENCE = 4
} led_sequence_t;

self_test_status_t startup_procedure ( void );
self_test_status_t initial_power_on_self_test ( void );

bool gnss_apply_config ( GNSS *gnss );
bool ct_init_and_self_test ( CT *ct );
bool temperature_init_and_self_test ( Temperature *temperature );
bool turbidity_init_and_self_test ( void );
bool light_init_and_self_test ( void );
bool accelerometer_init_and_self_test ( void );
bool waves_thread_init ( void );
bool iridium_init_and_config ( Iridium *iridium );

bool is_first_sample_window ( void );

void led_sequence ( uint8_t sequence );
void jump_to_end_of_window ( ULONG error_bits_to_set );
void jump_to_waves ( void );
void send_error_message ( ULONG error_flags );
void shut_it_all_down ( void );

#endif /* INC_THREAD_FUNCTIONS_H_ */

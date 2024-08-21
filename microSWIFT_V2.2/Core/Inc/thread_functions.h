/*
 * thread_functions.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef INC_THREAD_FUNCTIONS_H_
#define INC_THREAD_FUNCTIONS_H_

self_test_status_t startup_procedure ( void );
self_test_status_t initial_power_on_self_test ( void );
void led_sequence ( uint8_t sequence );
void jump_to_end_of_window ( ULONG error_bits_to_set );
void jump_to_waves ( void );
void send_error_message ( ULONG error_flags );
void shut_it_all_down ( void );

#endif /* INC_THREAD_FUNCTIONS_H_ */

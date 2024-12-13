/*
 * leds.h
 *
 *  Created on: Dec 13, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LEDS_H_
#define COMPONENTS_INC_LEDS_H_

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE = 1,
  TEST_PASSED_LED_SEQUENCE = 2,
  TEST_FAILED_LED_SEQUENCE = 3,
  HEARTBEAT_SEQUENCE = 4
} led_sequence_t;

void led_timer_expired ( ULONG expiration_input );

#endif /* COMPONENTS_INC_LEDS_H_ */

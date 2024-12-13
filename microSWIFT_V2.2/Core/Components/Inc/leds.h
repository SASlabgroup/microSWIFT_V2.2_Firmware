/*
 * leds.h
 *
 *  Created on: Dec 13, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LEDS_H_
#define COMPONENTS_INC_LEDS_H_

#include "stdbool.h"
#include "gpio.h"

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE = 1,
  TEST_PASSED_LED_SEQUENCE = 2,
  TEST_FAILED_LED_SEQUENCE = 3,
  HEARTBEAT_SEQUENCE = 4
} led_sequence_t;

void leds_init ( TX_TIMER *led_surartion_timer, gpio_pin_struct red_led, gpio_pin_struct green_led );

void led_timer_expired ( ULONG expiration_input );

#endif /* COMPONENTS_INC_LEDS_H_ */

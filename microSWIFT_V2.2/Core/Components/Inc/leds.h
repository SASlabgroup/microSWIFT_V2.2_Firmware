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
#include "tx_api.h"

#define HEARTBEAT_ON_TICKS 10U
#define HEARTBEAT_OFF_TICKS (TX_TIMER_TICKS_PER_SECOND - HEARTBEAT_ON_TICKS)
#define INITIAL_SEQUENCE_DELAY_TICKS (TX_TIMER_TICKS_PER_SECOND / 4U)
#define PASSED_SEQUENCE_DELAY_TICKS (TX_TIMER_TICKS_PER_SECOND)
#define FAILED_SEQUENCE_DELAY_TICKS (TX_TIMER_TICKS_PER_SECOND / 2U)

#define LED_MESSAGE_QUEUE_LENGTH 4

//@formatter:off
typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE      = 1,
  TEST_PASSED_LED_SEQUENCE  = 2,
  TEST_FAILED_LED_SEQUENCE  = 3,
  HEARTBEAT_SEQUENCE        = 4,
  LIGHTS_OFF                = 5,
  LIGHTS_ON                 = 6
} led_sequence_t;

typedef struct
{
  uint32_t          duration_sec;
  led_sequence_t    sequence;
} led_message;

typedef struct
{
  gpio_pin_struct   red_led;
  gpio_pin_struct   green_led;
  TX_TIMER          *duration_timer;

  bool              stop;
  bool              timer_timeout;

  led_sequence_t    current_sequence;
} LEDs;

void leds_init ( TX_TIMER *led_durartion_timer );
void leds_stop (void);
void led_timer_expired ( ULONG expiration_input );
void led_light_sequence ( led_sequence_t sequence, uint32_t duration );
//@formatter:on
#endif /* COMPONENTS_INC_LEDS_H_ */

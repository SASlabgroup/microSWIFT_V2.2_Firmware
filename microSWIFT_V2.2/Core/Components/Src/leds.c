/*
 * leds.c
 *
 *  Created on: Dec 13, 2024
 *      Author: philbush
 */

#include "leds.h"

/**
 * @brief
 *
 * @retval Void
 */
void led_sequence ( led_sequence_t sequence )
{
  switch ( sequence )
  {
    case INITIAL_LED_SEQUENCE:
      for ( int i = 0; i < 10; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
      }
      break;

    case TEST_PASSED_LED_SEQUENCE:
      for ( int i = 0; i < 5; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
      }
      break;

    case TEST_FAILED_LED_SEQUENCE:
      for ( int i = 0; i < 10; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 2);
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 2);
      }
      break;

    default:
      break;
  }
}

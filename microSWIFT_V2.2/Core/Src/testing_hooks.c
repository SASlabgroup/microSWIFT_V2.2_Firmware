/*
 * testing_hooks.c
 *
 *  Created on: Aug 1, 2024
 *      Author: philbush
 */

#include "testing_hooks.h"
#include "stddef.h"
#include "ext_rtc.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ext_psram.h"
bool test_psram ( void *unused );

bool test_rtc_int_mcu_reset ( void *unused );
static void __get_alarm_settings_from_time ( struct tm *time, rtc_alarm_struct *alarm );
/**************************************************************************************************/
/*********************************** Init --> Assign Tests ****************************************/
/**************************************************************************************************/
void tests_init ( void )
{
  tests.main_test = NULL;
  tests.threadx_init_test = NULL;
  tests.control_test = test_rtc_int_mcu_reset;
  tests.gnss_thread_test = NULL;
  tests.ct_thread_test = NULL;
  tests.light_thread_test = NULL;
  tests.turbidity_thread_test = NULL;
  tests.accelerometer_thread_test = NULL;
  tests.expansion_thread_1_test = NULL;
  tests.expansion_thread_2_test = NULL;
  tests.expansion_thread_3_test = NULL;
  tests.waves_thread_test = NULL;
  tests.iridium_thread_test = NULL;
  tests.shutdown_test = NULL;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Definitions ############################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

bool test_psram ( void *unused )
{
  const uint8_t test_str[] = "This is a test of the external PSRAM chip.";
  uint8_t read_buf[64] =
    { 0 };
  uint8_t data_len = strlen (test_str);
  uint8_t *psram_addr = (uint8_t*) OCTOSPI1_BASE;

  if ( !initialize_psram () )
  {
    return false;
  }

  // Test write
  for ( int i = 0; i < data_len; i++, psram_addr++ )
  {
    *psram_addr = test_str[i];
  }

  psram_addr = (uint8_t*) OCTOSPI1_BASE;

  // Test read
  for ( int i = 0; i < data_len; i++ )
  {
    read_buf[i] = *psram_addr;
  }

  return (strcmp (test_str, read_buf) == 0);
}

bool test_rtc_int_mcu_reset ( void *unused )
{
  rtc_alarm_struct alarm_settings;
  struct tm time_now;
  uint8_t dummy;

  // Set the GPIO pin low for the OR logic gate
  HAL_GPIO_WritePin (GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  tx_thread_sleep (100);
  // Get the time so we can set the alarm
  if ( rtc_server_get_time (&time_now, CONTROL_REQUEST_COMPLETE) != uSWIFT_SUCCESS )
  {
    HAL_NVIC_SystemReset ();
  }

  // Set the alarm
  __get_alarm_settings_from_time (&time_now, &alarm_settings);
  if ( rtc_server_set_alarm (alarm_settings, CONTROL_REQUEST_COMPLETE) != uSWIFT_SUCCESS )
  {
    HAL_NVIC_SystemReset ();
  }

  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND * 60);

  dummy = 0;
}

static void __get_alarm_settings_from_time ( struct tm *time, rtc_alarm_struct *alarm )
{
  alarm->day_alarm_en = false;
  alarm->hour_alarm_en = true;
  alarm->minute_alarm_en = true;
  alarm->second_alarm_en = true;
  alarm->weekday_alarm_en = false;
  alarm->alarm_second = 0;

  alarm->alarm_second = (time->tm_sec + 10) % 60;
  alarm->alarm_minute = (alarm->alarm_second < time->tm_sec) ?
      (time->tm_min + 1) % 60 : time->tm_min;
  alarm->alarm_hour = (alarm->alarm_minute == 0) ?
      (time->tm_hour + 1) % 24 : time->tm_hour;
  return;
}

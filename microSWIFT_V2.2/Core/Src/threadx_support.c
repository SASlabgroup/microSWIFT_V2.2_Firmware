/*
 * thread_functions.c
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#include "threadx_support.h"
#include "stdbool.h"
#include "stdarg.h"
#include "stdio.h"
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
#include "persistent_ram.h"
#include "logger.h"
#include "watchdog.h"

bool gnss_apply_config ( GNSS *gnss )
{
  int fail_counter = 0, max_retries = 20;
  gnss_return_code_t gnss_return_code = GNSS_SUCCESS;

  while ( fail_counter < max_retries )
  {
    gnss_return_code = gnss->config ();

    if ( gnss_return_code == GNSS_SUCCESS )
    {
      break;
    }

    // After 5 failures, power cycle the GNSS
    if ( fail_counter % 5 == 0 )
    {
      gnss->off ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
      gnss->on ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;
      continue;
    }

    tx_thread_sleep (10);
    fail_counter++;
  }

  return (gnss_return_code == GNSS_SUCCESS);
}

bool ct_self_test ( CT *ct, bool add_warmup_time, ct_sample *self_test_readings )
{
  int32_t fail_counter = 0, max_retries = 10;
  ct_return_code_t ct_return_code;

  while ( fail_counter < max_retries )
  {
    ct_return_code = ct->self_test (add_warmup_time, self_test_readings);

    if ( ct_return_code == CT_SUCCESS )
    {
      break;
    }

    ct->off ();
    tx_thread_sleep (1);
    ct->on ();
    tx_thread_sleep (1);

    fail_counter++;
  }

  return (ct_return_code == CT_SUCCESS);
}

bool temperature_self_test ( Temperature *temperature, float *self_test_temp )
{
  int32_t fail_counter = 0, max_retries = 10;
  temperature_return_code_t temp_return_code;

  while ( fail_counter < max_retries )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    temp_return_code = temperature->self_test (self_test_temp);

    if ( temp_return_code == TEMPERATURE_SUCCESS )
    {
      break;
    }

    temperature->off ();
    tx_thread_sleep (1);
    temperature->on ();
    tx_thread_sleep (1);

    fail_counter++;
  }

  return (temp_return_code == TEMPERATURE_SUCCESS);
}

bool turbidity_self_test ( void )
{
  return true;
}

bool light_self_test ( Light_Sensor *light )
{
  int32_t fail_counter = 0, max_retries = 10;
  light_return_code_t light_return_code;

  while ( fail_counter < max_retries )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    light_return_code = light->self_test ();

    if ( light_return_code == LIGHT_SUCCESS )
    {
      break;
    }

    light->off ();
    tx_thread_sleep (1);
    light->on ();
    tx_thread_sleep (1);

    fail_counter++;
  }

  return (light_return_code == LIGHT_SUCCESS);
}

bool accelerometer_self_test ( void )
{
  return true;
}

bool iridium_apply_config ( Iridium *iridium )
{
  int32_t fail_counter = 0, max_retries = 10;
  iridium_return_code_t iridium_return_code;

  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->self_test ();

    if ( iridium_return_code == IRIDIUM_SUCCESS )
    {
      break;
    }

    iridium->off ();
    tx_thread_sleep (25);
    iridium->on ();
    tx_thread_sleep (25);
    fail_counter++;
  }

  if ( iridium_return_code != IRIDIUM_SUCCESS )
  {
    return false;
  }

  // Send the configuration settings to the modem
  fail_counter = 0;
  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->config ();

    if ( iridium_return_code == IRIDIUM_SUCCESS )
    {
      break;
    }

    iridium->off ();
    tx_thread_sleep (1);
    iridium->on ();
    tx_thread_sleep (1);
    fail_counter++;
  }

  return (iridium_return_code == IRIDIUM_SUCCESS);
}

void gnss_error_out ( GNSS *gnss, ULONG error_flag, TX_THREAD *gnss_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  gnss->off ();
  gnss->stop_timer ();
  gnss_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (GNSS_THREAD);

  tx_thread_suspend (gnss_thread);
}

void ct_error_out ( CT *ct, ULONG error_flag, TX_THREAD *ct_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  ct->off ();
  ct->stop_timer ();
  ct_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (CT_THREAD);

  tx_thread_suspend (ct_thread);
}

void temperature_error_out ( Temperature *temperature, ULONG error_flag,
                             TX_THREAD *temperature_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  temperature->off ();
  temperature->stop_timer ();
  temperature_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (TEMPERATURE_THREAD);

  tx_thread_suspend (temperature_thread);
}

void light_error_out ( Light_Sensor *light, ULONG error_flag, TX_THREAD *light_thread,
                       const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  light->off ();
  light->stop_timer ();
  light_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (LIGHT_THREAD);

  tx_thread_suspend (light_thread);
}

void waves_error_out ( ULONG error_flag, TX_THREAD *waves_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, WAVES_INIT_FAILED, TX_OR);

  watchdog_deregister_thread (WAVES_THREAD);

  tx_thread_suspend (waves_thread);
}

void iridium_error_out ( Iridium *iridium, ULONG error_flag, TX_THREAD *iridium_thread,
                         const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  iridium->sleep ();
  iridium->off ();
  iridium->stop_timer ();
  iridium_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (IRIDIUM_THREAD);

  tx_thread_suspend (iridium_thread);
}

void rtc_error_out ( TX_THREAD *rtc_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, RTC_ERROR, TX_OR);

  tx_thread_suspend (rtc_thread);
}

void filex_error_out ( TX_THREAD *filex_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, FILE_SYSTEM_ERROR, TX_OR);

  tx_thread_suspend (filex_thread);
}

bool is_first_sample_window ( void )
{
  return (persistent_ram_get_sample_window_counter () == 0);
}

ULONG get_current_flags ( TX_EVENT_FLAGS_GROUP *event_flags )
{
  ULONG current_flags = 0;
  (void) tx_event_flags_get (event_flags, ALL_EVENT_FLAGS, TX_OR, &current_flags, TX_NO_WAIT);
  return current_flags;
}

void clear_event_flags ( TX_EVENT_FLAGS_GROUP *event_flags )
{
  ULONG current_flags = 0;
  (void) tx_event_flags_get (event_flags, ALL_EVENT_FLAGS, TX_OR_CLEAR, &current_flags, TX_NO_WAIT);
}

uint32_t ticks_from_milliseconds ( uint32_t milliseconds )
{
  if ( milliseconds == 0 )
  {
    return 0;
  }

  return ((uint32_t) (ceil (((float) milliseconds) / ((1.0f / TX_TIMER_TICKS_PER_SECOND) * 1000.0f))));
}

/**
 * @brief  Static function to flash a sequence of onboard LEDs to indicate
 * success or failure of self-test.
 *
 * @param  sequence:   INITIAL_LED_SEQUENCE
 *                                     TEST_PASSED_LED_SEQUENCE
 *                                     TEST_NON_CIRTICAL_FAULT_LED_SEQUENCE
 *                                     TEST_CRITICAL_FAULT_LED_SEQUENCE
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

/**
 * @brief  If an error was detected along the way, send an error (Type 99) message.
 *
 * @param  error_flags - retreived error flags
 *
 * @retval void
 */
void send_error_message ( Iridium *iridium, ULONG error_flags )
{
//  iridium_error_code_t return_code;
//  char error_message[ERROR_MESSAGE_MAX_LENGTH] =
//    { 0 };
//  const char *watchdog_reset = "WATCHDOG RESET. ";
//  const char *software_reset = "SOFTWARE RESET. ";
//  const char *gnss_error = "GNSS ERROR. ";
//  const char *gnss_resolution_error = "GNSS RESOLUTION ERROR. ";
//  const char *sample_window_error = "SAMPLE WINDOW ERROR. ";
//  const char *memory_corruption_error = "MEMORY CORRUPTION ERROR. ";
//  const char *memory_alloc_error = "MEMORY ALLOC ERROR. ";
//  const char *dma_error = "DMA ERROR. ";
//  const char *uart_error = "UART ERROR. ";
//  const char *rtc_error = "RTC ERROR. ";
//  const char *flash_success = "WRITE TO FLASH SUCCESSFUL. ";
//  const char *flash_unknown_error = "UNKNOWN FLASH ERROR. ";
//  const char *flash_storage_full = "FLASH STORAGE FULL. ";
//  const char *flash_erase_error = "FLASH ERASE ERROR. ";
//  const char *flash_program_error = "FLASH PROGRAM ERROR. ";
//  char *string_ptr = &(error_message[0]);
//
//  if ( error_flags & WATCHDOG_RESET )
//  {
//
//    if ( (string_ptr - &(error_message[0])) + strlen (watchdog_reset) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, watchdog_reset, strlen (watchdog_reset));
//      string_ptr += strlen (watchdog_reset);
//    }
//
//  }
//
//  if ( error_flags & SOFTWARE_RESET )
//  {
//
//    if ( (string_ptr - &(error_message[0])) + strlen (software_reset) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, software_reset, strlen (software_reset));
//      string_ptr += strlen (software_reset);
//    }
//
//  }
//
//  if ( error_flags & GNSS_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (gnss_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, gnss_error, strlen (gnss_error));
//      string_ptr += strlen (gnss_error);
//    }
//  }
//
//  if ( error_flags & GNSS_RESOLUTION_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (gnss_resolution_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, gnss_resolution_error, strlen (gnss_resolution_error));
//      string_ptr += strlen (gnss_resolution_error);
//    }
//  }
//
//  if ( error_flags & SAMPLE_WINDOW_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (sample_window_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, sample_window_error, strlen (sample_window_error));
//      string_ptr += strlen (sample_window_error);
//    }
//  }
//
//  if ( error_flags & MEMORY_CORRUPTION_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (memory_corruption_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, memory_corruption_error, strlen (memory_corruption_error));
//      string_ptr += strlen (memory_corruption_error);
//    }
//  }
//
//  if ( error_flags & MEMORY_ALLOC_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (memory_alloc_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, memory_alloc_error, strlen (memory_alloc_error));
//      string_ptr += strlen (memory_alloc_error);
//    }
//  }
//
//  if ( error_flags & DMA_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (dma_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, dma_error, strlen (dma_error));
//      string_ptr += strlen (dma_error);
//    }
//  }
//
//  if ( error_flags & UART_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (uart_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, uart_error, strlen (uart_error));
//      string_ptr += strlen (uart_error);
//    }
//  }
//
//  if ( error_flags & RTC_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (rtc_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, rtc_error, strlen (rtc_error));
//      string_ptr += strlen (rtc_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_SUCCESS )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (flash_success) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_success, strlen (flash_success));
//      string_ptr += strlen (flash_success);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_UNKNOWN_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_unknown_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_unknown_error, strlen (flash_unknown_error));
//      string_ptr += strlen (flash_unknown_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_STORAGE_FULL )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_storage_full) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_storage_full, strlen (flash_storage_full));
//      string_ptr += strlen (flash_storage_full);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_ERASE_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (flash_erase_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_erase_error, strlen (flash_erase_error));
//      string_ptr += strlen (flash_erase_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_PROGRAM_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_program_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_program_error, strlen (flash_program_error));
//      string_ptr += strlen (flash_program_error);
//    }
//  }
//
//  return_code = iridium->transmit_error_message (error_message);
//
//  if ( (return_code == IRIDIUM_SUCCESS) && (device_handles.reset_reason != 0) )
//  {
//    // Only want to send this message once, so clear reset_reason
//    device_handles.reset_reason = 0;
//  }
}


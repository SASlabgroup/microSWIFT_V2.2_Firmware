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
#include "iridium.h"
#include "persistent_ram.h"
#include "logger.h"
#include "watchdog.h"
#include "sdmmc.h"

bool gnss_apply_config ( GNSS *gnss )
{
  int fail_counter = 0, max_retries = 20;
  uSWIFT_return_code_t gnss_return_code = uSWIFT_SUCCESS;

  while ( fail_counter < max_retries )
  {
    gnss_return_code = gnss->config ();

    if ( gnss_return_code == uSWIFT_SUCCESS )
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

    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    fail_counter++;
  }

  return (gnss_return_code == uSWIFT_SUCCESS);
}

bool ct_self_test ( CT *ct, bool add_warmup_time, ct_sample *self_test_readings )
{
  int32_t fail_counter = 0, max_retries = 10;
  uSWIFT_return_code_t ct_return_code;

  while ( fail_counter < max_retries )
  {
    ct_return_code = ct->self_test (add_warmup_time, self_test_readings);

    if ( ct_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    ct->off ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    ct->on ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    fail_counter++;
  }

  return (ct_return_code == uSWIFT_SUCCESS);
}

bool temperature_self_test ( Temperature *temperature, float *self_test_temp )
{
  int32_t fail_counter = 0, max_retries = 10;
  uSWIFT_return_code_t temp_return_code;

  while ( fail_counter < max_retries )
  {
    temp_return_code = temperature->self_test (self_test_temp);

    if ( temp_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    fail_counter++;
  }

  return (temp_return_code == uSWIFT_SUCCESS);
}

bool turbidity_self_test ( Turbidity_Sensor *obs )
{
  int32_t fail_counter = 0, max_retries = 10;
  uSWIFT_return_code_t ret;

  while ( fail_counter < max_retries )
  {
    ret = obs->self_test ();

    if ( ret == uSWIFT_SUCCESS )
    {
      break;
    }

    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    fail_counter++;
  }

  return (ret == uSWIFT_SUCCESS);
}

bool light_self_test ( Light_Sensor *light )
{
  int32_t fail_counter = 0, max_retries = 10;
  uSWIFT_return_code_t light_return_code;

  while ( fail_counter < max_retries )
  {
    light_return_code = light->self_test ();

    if ( light_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

    fail_counter++;
  }

  return (light_return_code == uSWIFT_SUCCESS);
}

bool iridium_apply_config ( Iridium *iridium )
{
  int32_t fail_counter = 0, max_retries = 10;
  uSWIFT_return_code_t iridium_return_code;

  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->self_test ();

    if ( iridium_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    iridium->off ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
    iridium->on ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
    fail_counter++;
  }

  if ( iridium_return_code != uSWIFT_SUCCESS )
  {
    return false;
  }

  // Send the configuration settings to the modem
  fail_counter = 0;
  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->config ();

    if ( iridium_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    iridium->off ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
    iridium->on ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
    fail_counter++;
  }

  return (iridium_return_code == uSWIFT_SUCCESS);
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

  light->idle ();
  light->stop_timer ();
  light_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (LIGHT_THREAD);

  tx_thread_suspend (light_thread);
}

void turbidity_error_out ( Turbidity_Sensor *turbidity, ULONG error_flag,
                           TX_THREAD *turbidity_thread, const char *fmt, ... )
{
  va_list args;
  va_start(args, fmt);
  char tmp_fmt[128];

  turbidity->idle ();
  turbidity->stop_timer ();
  turbidity_deinit ();

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, error_flag, TX_OR);

  watchdog_deregister_thread (TURBIDITY_THREAD);

  tx_thread_suspend (turbidity_thread);
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

  HAL_SD_DeInit (&hsd2);

  vsnprintf (&tmp_fmt[0], sizeof(tmp_fmt), fmt, args);
  va_end(args);
  LOG(&(tmp_fmt[0]));

  (void) tx_event_flags_set (&error_flags, FILE_SYSTEM_ERROR, TX_OR);

  tx_thread_suspend (filex_thread);
}

bool get_next_telemetry_message ( uint8_t *msg_buffer, microSWIFT_configuration config )
{
#error "Figure this out."
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


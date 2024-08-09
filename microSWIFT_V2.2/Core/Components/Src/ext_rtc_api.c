/*
 * ext_rtc_api.c
 *
 *  Created on: Aug 9, 2024
 *      Author: philbush
 */

#include "ext_rtc_api.h"

static rtc_server self;

void rtc_server_init ( TX_QUEUE *request_queue, TX_SEMAPHORE *watchdog_refresh_semaphore,
                       TX_EVENT_FLAGS_GROUP *complete_flags )
{
  self.request_queue = request_queue;
  self.watchdog_refresh_semaphore = watchdog_refresh_semaphore;
  self.complete_flags = complete_flags;
}

void rtc_server_refresh_watchdog ( void )
{
  // The RTC thread will monitor the semaphore, whenever there is a put, it will get and refresh
  (void) tx_semaphore_put (self.watchdog_refresh_semaphore);
}

ext_rtc_return_code rtc_server_get_time ( struct tm *return_time_struct, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  queue_msg.request = GET_TIME;
  queue_msg.input_output_struct = return_time_struct;
  queue_msg.complete_flag = complete_flag;
  queue_msg.return_code = &ret;

  if ( tx_queue_send (self.request_queue, &queue_msg, RTC_QUEUE_MAX_WAIT_TICKS) != TX_SUCCESS )
  {
    return RTC_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (self.complete_flags, complete_flag, TX_OR_CLEAR, &event_flags,
                           RTC_FLAG_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return RTC_TIMEOUT;
  }

  return ret;
}

ext_rtc_return_code rtc_server_set_time ( struct tm *input_time_struct, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  return ret;
}

ext_rtc_return_code rtc_server_config_watchdog ( uint32_t period_ms, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  return ret;
}

ext_rtc_return_code rtc_server_set_timestamp ( rtc_timestamp_t which_timestamp, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  return ret;
}

ext_rtc_return_code rtc_server_get_timestamp ( rtc_timestamp_t which_timestamp, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  return ret;
}

ext_rtc_return_code rtc_server_set_alarm ( rtc_set_alarm_t *alarm_settings, UINT complete_flag )
{
  ext_rtc_return_code ret = RTC_SUCCESS;
  rtc_request_message queue_msg;
  ULONG event_flags;

  return ret;
}


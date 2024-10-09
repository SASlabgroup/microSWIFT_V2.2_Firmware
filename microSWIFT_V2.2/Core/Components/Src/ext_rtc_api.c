/*
 * ext_rtc_api.c
 *
 *  Created on: Aug 9, 2024
 *      Author: philbush
 */

#include "ext_rtc_api.h"

static rtc_server self;

void rtc_server_init ( TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags )
{
  self.request_queue = request_queue;
  self.complete_flags = complete_flags;
}

void rtc_server_refresh_watchdog ( void )
{
  rtc_request_message queue_msg =
    { 0 };

  queue_msg.request = REFRESH_WATCHDOG;
  queue_msg.complete_flag = 0;
  queue_msg.return_code = NULL;

  (void) tx_queue_send (self.request_queue, &queue_msg, RTC_QUEUE_MAX_WAIT_TICKS);
}

rtc_return_code rtc_server_get_time ( struct tm *return_time_struct, UINT complete_flag )
{
  int32_t ret = RTC_SUCCESS;
  rtc_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = GET_TIME;
  queue_msg.input_output_struct.get_set_time.time_struct = *return_time_struct;
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

rtc_return_code rtc_server_set_time ( struct tm input_time_struct, UINT complete_flag )
{
  int32_t ret = RTC_SUCCESS;
  rtc_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SET_TIME;
  queue_msg.input_output_struct.get_set_time.time_struct = input_time_struct;
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

rtc_return_code rtc_server_set_timestamp ( pcf2131_timestamp_t which_timestamp, UINT complete_flag )
{
  int32_t ret = RTC_SUCCESS;
  rtc_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SET_TIMESTAMP;
  queue_msg.input_output_struct.get_set_timestamp.which_timestamp = which_timestamp;
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

rtc_return_code rtc_server_get_timestamp ( pcf2131_timestamp_t which_timestamp, UINT complete_flag )
{
  int32_t ret = RTC_SUCCESS;
  rtc_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = GET_TIMESTAMP;
  queue_msg.input_output_struct.get_set_timestamp.which_timestamp = which_timestamp;
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

rtc_return_code rtc_server_set_alarm ( rtc_alarm_struct alarm_settings, UINT complete_flag )
{
  int32_t ret = RTC_SUCCESS;
  rtc_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SET_ALARM;
  queue_msg.input_output_struct.set_alarm = alarm_settings;
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

// Requires request message to be filled out
rtc_return_code rtc_server_process_request ( rtc_request_message *request )
{
  int32_t ret = RTC_SUCCESS;
  ULONG event_flags;

  if ( tx_queue_send (self.request_queue, request, RTC_QUEUE_MAX_WAIT_TICKS) != TX_SUCCESS )
  {
    return RTC_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (self.complete_flags, request->complete_flag, TX_OR_CLEAR, &event_flags,
  RTC_FLAG_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return RTC_TIMEOUT;
  }

  return ret;
}

void struct_tm_dec_to_bcd ( struct tm *struct_ptr )
{
  struct_ptr->tm_sec = (struct_ptr->tm_sec > 59) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_sec];
  struct_ptr->tm_min = (struct_ptr->tm_min > 59) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_min];
  struct_ptr->tm_hour = (struct_ptr->tm_hour > 23) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_hour];
  struct_ptr->tm_mday = (struct_ptr->tm_mday > 31) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_mday];
  struct_ptr->tm_mon = (struct_ptr->tm_mon > 11) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_mon];
  struct_ptr->tm_year = (struct_ptr->tm_year - 2000 > 99) ?
      BCD_ERROR : struct_ptr->tm_year - 2000;
  struct_ptr->tm_wday = (struct_ptr->tm_wday > 6) ?
      BCD_ERROR : dec_to_bcd[struct_ptr->tm_wday];
}

// Not guaranteed to be safe...
void struct_tm_bcd_to_dec ( struct tm *struct_ptr )
{
  struct_ptr->tm_sec = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_min = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_hour = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_mday = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_mon = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_year = bcd_to_dec[struct_ptr->tm_sec];
  struct_ptr->tm_wday = bcd_to_dec[struct_ptr->tm_sec];
}


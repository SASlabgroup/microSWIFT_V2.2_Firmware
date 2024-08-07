/*
 * ext_rtc_api.h
 *
 *  Created on: Aug 6, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_EXT_RTC_API_H_
#define COMPONENTS_INC_EXT_RTC_API_H_

#include "stddef.h"
#include "time.h"
#include "pcf2131_reg.h"

typedef enum
{
  GET_TIME = 0,
  SET_TIME_DATE = 1,
  CONFIG_WATCHDOG = 2,
  REFRESH_WATCHDOG = 3,
  GET_TIMESTAMP = 4,
  SET_ALARM = 5
} rtc_request_t;

typedef enum
{
  TIMESTAMP_1 = 1,
  TIMESTAMP_2 = 2,
  TIMESTAMP_3 = 3,
  TIMESTAMP_4 = 4
} rtc_timestamp_t;

typedef struct
{
  struct tm time_struct;
} rtc_get_time_t;

typedef struct
{
  struct tm time_struct;
} rtc_set_time_t;

typedef struct
{
  uint32_t period_ms;
} rtc_config_watchdog_t;

typedef struct
{
  rtc_timestamp_t which_timestamp;
  time_t timestamp;
} rtc_get_timestamp_t;

typedef struct
{
  uint8_t alarm_second;
  uint8_t alarm_minute;
  uint8_t alarm_hour;
  uint8_t alarm_day;
  weekday_t alarm_weekday;
  bool second_alarm_en;
  bool minute_alarm_en;
  bool hour_alarm_en;
  bool day_alarm_en;
  bool weekday_alarm_en;
} rtc_alarm_struct;

typedef struct
{
  rtc_alarm_struct alarm_config;
} rtc_set_alarm_t;

typedef union
{
  rtc_get_time_t get_time;
  rtc_set_time_t set_time;
  rtc_config_watchdog_t config_watchdog;
  rtc_get_timestamp_t get_timestamp;
  rtc_set_alarm_t set_alarm;
} rtc_data_t;

typedef struct
{
  rtc_request_t request :32;
  rtc_data_t *input_parameters;
  UINT complete_flag;
  int32_t *return_code;
} rtc_request_message;

#endif /* COMPONENTS_INC_EXT_RTC_API_H_ */

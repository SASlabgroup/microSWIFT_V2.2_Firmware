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
#include "pcf2131.h"

typedef enum
{
  CONTROL = 0,
  GNSS = 1,
  IRIDIUM = 2,
  CT = 3,
  TEMP = 4,
  LIGHT = 5,
  TURBIDITY = 6,
  MISSION_BOARD_1 = 7,
  MISSION_BOARD_2 = 8,
  MISSION_BOARD_3 = 9,
  MISSION_BOARD_4 = 10,
  MISSION_BOARD_5 = 11,
  MISSION_BOARD_6 = 12,
  MISSION_BOARD_7 = 13,
  MISSION_BOARD_8 = 14
} rtc_requestor_t;

typedef enum
{
  GET_TIME = 0,
  SET_TIME_DATE = 1,
  CONFIG_WATCHDOG = 2,
  REFRESH_WATCHDOG = 3,
  GET_TIMESTAMP = 4,
  SET_ALARM = 5
} rtc_command_t;

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
  rtc_requestor requestor;
  rtc_command_t command;
  rtc_data_t input_parameters;
} rtc_request_message;

typedef struct
{
  rtc_requestor recipient;
  rtc_command_t command;
  rtc_data_t requested_data;
} rtc_response_message;

typedef union
{
  rtc_request_message request;
  rtc_response_message response;
} rtc_message;

#endif /* COMPONENTS_INC_EXT_RTC_API_H_ */

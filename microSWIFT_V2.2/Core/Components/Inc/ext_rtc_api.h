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
#include "limits.h"

#define RTC_QUEUE_MAX_WAIT_TICKS 5
#define RTC_FLAG_MAX_WAIT_TICKS 5

// @formatter:off
/* Looku table for fast conversion from BCD to decimal */
uint8_t bcd_to_dec[256] =
{
  // 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0, 1, 2, 3, 4, 5, 6, 7,
  // 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  8, 9, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  10, 11, 12, 13, 14, 15, 16, 17,
  // 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
  18, 19, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
  20, 21, 22, 23, 24, 25, 26, 27,
  // 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
  28, 29, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
  30, 31, 32, 33, 34, 35, 36, 37,
  // 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
  38, 39, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
  40, 41, 42, 43, 44, 45, 46, 47,
  // 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
  48, 49, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
  50, 51, 52, 53, 54, 55, 56, 57,
  // 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
  58, 59, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
  60, 61, 62, 63, 64, 65, 66, 67,
  // 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
  68, 69, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
  70, 71, 72, 73, 74, 75, 76, 77,
  // 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
  78, 79, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
  80, 81, 82, 83, 84, 85, 86, 87,
  // 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
  88, 89, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
  90, 91, 92, 93, 94, 95, 96, 97,
  // 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
  98, 99, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7,
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
  // 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
  BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR
};

uint8_t dec_to_bcd[100] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, // 0-9
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, // 10-19
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, // 20-29
  0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, // 30-39
  0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, // 40-49
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, // 50-59
  0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, // 60-69
  0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, // 70-79
  0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, // 80-89
  0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99  // 90-99
};
// @formatter:on

// Which request function does a requester want performed
typedef enum
{
  GET_TIME = 0,
  SET_TIME = 1,
  CONFIG_WATCHDOG = 2,
  SET_TIMESTAMP = 3,
  GET_TIMESTAMP = 4,
  SET_ALARM = 5
} rtc_request_t;

// Event group flags for signalling completion
typedef enum
{
  CONTROL_THREAD_REQUEST_PROCESSED = (1 << 0),
  GNSS_REQUEST_PROCESSED = (1 << 1),
  CT_REQUEST_PROCESSED = (1 << 2),
  TEMPERATURE_REQUEST_PROCESSED = (1 << 3),
  LIGHT_REQUEST_PROCESSED = (1 << 4),
  TURBIDITY_REQUEST_PROCESSED = (1 << 5),
  IRIDIUM_REQUEST_PROCESSED = (1 << 6),
  AUX_PERIPHERAL_1_REQUEST_PROCESSED = (1 << 7)
};

// Define the timestamps
typedef enum
{
  TIMESTAMP_1 = 1,
  TIMESTAMP_2 = 2,
  TIMESTAMP_3 = 3,
  TIMESTAMP_4 = 4
} rtc_timestamp_t;

// Message struct for GET_TIME request
typedef struct
{
  struct tm time_struct;
} rtc_get_time_t;

// Message struct for SET_TIME request
typedef struct
{
  struct tm time_struct;
} rtc_set_time_t;

// Message struct for CONFIG_WATCHDOG request
typedef struct
{
  uint32_t period_ms;
} rtc_config_watchdog_t;

// Message struct for GET_TIMESTAMP request
typedef struct
{
  rtc_timestamp_t which_timestamp;
  time_t timestamp;
} rtc_get_timestamp_t;

// Message struct for SET_ALARM request
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
} rtc_set_alarm_t;

// Generic message type for request -- used to create uniform request size
typedef union
{
  rtc_get_time_t get_time;
  rtc_set_time_t set_time;
  rtc_config_watchdog_t config_watchdog;
  rtc_get_timestamp_t get_timestamp;
  rtc_set_alarm_t set_alarm;
} rtc_data_t;

// Generic message request format
typedef struct
{
  int32_t request; // Use type rtc_request_t
  // For set requests, this contains input data
  // For get requests, this is written back to
  rtc_data_t *input_output_struct;
  UINT complete_flag;
  int32_t *return_code;
} rtc_request_message;

typedef struct
{
  TX_QUEUE *request_queue;
  TX_SEMAPHORE *watchdog_refresh_semaphore;
  TX_EVENT_FLAGS_GROUP *complete_flags;
} rtc_server;

// Interface functions
void rtc_server_init ( TX_QUEUE *request_queue, TX_SEMAPHORE *watchdog_refresh_semaphore );
void rtc_server_refresh_watchdog ( void );
ext_rtc_return_code rtc_server_get_time ( struct tm *return_time_struct, UINT complete_flag );
ext_rtc_return_code rtc_server_set_time ( struct tm *input_time_struct, UINT complete_flag );
ext_rtc_return_code rtc_server_config_watchdog ( uint32_t *period_ms, UINT complete_flag );
ext_rtc_return_code rtc_server_set_timestamp ( rtc_timestamp_t *which_timestamp,
                                               UINT complete_flag );
ext_rtc_return_code rtc_server_get_timestamp ( rtc_timestamp_t *which_timestamp,
                                               UINT complete_flag );
ext_rtc_return_code rtc_server_set_alarm ( rtc_set_alarm_t *alarm_settings, UINT complete_flag );

#endif /* COMPONENTS_INC_EXT_RTC_API_H_ */

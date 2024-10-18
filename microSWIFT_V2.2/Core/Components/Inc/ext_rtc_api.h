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
#include "tx_api.h"

// @formatter:off

#define RTC_QUEUE_LENGTH            16U
#define RTC_QUEUE_MAX_WAIT_TICKS    2U
#define RTC_FLAG_MAX_WAIT_TICKS     10U

typedef enum
{
  RTC_SUCCESS                   = 0,
  RTC_SPI_ERROR                 = -1,
  RTC_PARAMETERS_INVALID        = -2,
  RTC_TIMESTAMP_ALREADY_IN_USE  = -3,
  RTC_TIMESTAMP_NOT_SET         = -4,
  RTC_MESSAGE_QUEUE_ERROR       = -5,
  RTC_TIMEOUT                   = -6
} rtc_return_code;

// Which request function does a requester want performed
typedef enum
{
  REFRESH_WATCHDOG  = 0,
  GET_TIME          = 1,
  SET_TIME          = 2,
  SET_TIMESTAMP     = 3,
  GET_TIMESTAMP     = 4,
  SET_ALARM         = 5,
  CLEAR_FLAG        = 6
} rtc_request_t;

typedef enum
{
  MINUTE_SECOND_FLAG        = 0,
  WATCHDOG_FLAG             = 1,
  ALARM_FLAG                = 2,
  BATTERY_SWITCHOVER_FLAG   = 3,
  BATTERY_STATUS_FLAG       = 4,
  TIMESTAMP1_FLAG           = 5,
  TIMESTAMP2_FLAG           = 6,
  TIMESTAMP3_FLAG           = 7,
  TIMESTAMP4_FLAG           = 8,
  ALL_RTC_FLAGS             = 9
} rtc_flag_t;

// Event group flags for signalling completion
typedef enum
{
  CONTROL_THREAD_REQUEST_PROCESSED      = ((ULONG) 1 << 1),
  GNSS_REQUEST_PROCESSED                = ((ULONG) 1 << 2),
  CT_REQUEST_PROCESSED                  = ((ULONG) 1 << 3),
  TEMPERATURE_REQUEST_PROCESSED         = ((ULONG) 1 << 4),
  LIGHT_REQUEST_PROCESSED               = ((ULONG) 1 << 5),
  TURBIDITY_REQUEST_PROCESSED           = ((ULONG) 1 << 6),
  IRIDIUM_REQUEST_PROCESSED             = ((ULONG) 1 << 7),
  LOGGER_REQUEST_PROCESSED              = ((ULONG) 1 << 8),
  AUX_PERIPHERAL_1_REQUEST_PROCESSED    = ((ULONG) 1 << 9)
} rtc_complete_flags_t;

// Message struct for GET_TIME request
typedef struct
{
  struct tm *time_struct;
} rtc_get_time_t;

// Message struct for SET_TIME request
typedef struct
{
  struct tm *time_struct;
} rtc_set_time_t;

// Message struct for GET_TIMESTAMP request
typedef struct
{
  pcf2131_timestamp_t   which_timestamp;
  time_t                timestamp;
} rtc_get_timestamp_t;

// Generic message type for request -- used to create uniform request size
typedef union
{
  rtc_get_time_t        get_set_time;
  rtc_get_timestamp_t   get_set_timestamp;
  rtc_alarm_struct      set_alarm;
  uint32_t              clear_flag;
} rtc_data_t;

// Generic message request format
typedef struct
{
  int32_t       request; // Use type rtc_request_t

  // For set requests, this contains input data
  // For get requests, this is written back to
  rtc_data_t    input_output_struct;
  UINT          complete_flag;
  int32_t       *return_code;
} rtc_request_message;

typedef struct
{
  TX_QUEUE              *request_queue;
  TX_SEMAPHORE          *watchdog_refresh_semaphore;
  TX_EVENT_FLAGS_GROUP  *complete_flags;
} rtc_server;

// Interface functions
void            rtc_server_init ( TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags );
void            rtc_server_refresh_watchdog ( void );
rtc_return_code rtc_server_get_time ( struct tm *return_time_struct, UINT complete_flag );
rtc_return_code rtc_server_set_time ( struct tm *input_time_struct, UINT complete_flag );
rtc_return_code rtc_server_set_timestamp ( pcf2131_timestamp_t which_timestamp, UINT complete_flag );
rtc_return_code rtc_server_get_timestamp ( pcf2131_timestamp_t which_timestamp, UINT complete_flag );
rtc_return_code rtc_server_set_alarm ( rtc_alarm_struct alarm_settings, UINT complete_flag );
rtc_return_code rtc_server_clear_flag (rtc_flag_t which_flag, UINT complete_flag );
// Generic do-all function
rtc_return_code rtc_server_process_request ( rtc_request_message *request );

/* Helper functions */
void            struct_tm_dec_to_bcd ( struct tm *struct_ptr );
void            struct_tm_bcd_to_dec ( struct tm *struct_ptr );

// @formatter:on

#endif /* COMPONENTS_INC_EXT_RTC_API_H_ */

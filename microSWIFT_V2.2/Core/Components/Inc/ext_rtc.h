/*
 * ext_rtc.h
 *
 *  Created on: Jul 29, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_EXT_RTC_H_
#define COMPONENTS_INC_EXT_RTC_H_

#include "tx_api.h"
#include "ext_rtc_api.h"
#include "spi.h"
#include "time.h"
#include "pcf2131_reg.h"
#include "gpio.h"

#define RTC_SPI_TIMEOUT 25
#define RTC_SPI_BUF_SIZE 8

typedef enum
{
  EXT_RTC_SUCCESS = 0,
  EXT_RTC_SPI_ERROR = -1,
  EXT_RTC_PARAMETERS_INVALID = -2,
  EXT_RTC_TIMESTAMP_ALREADY_IN_USE = -3,
  EXT_RTC_MESSAGE_QUEUE_ERROR = -4,
  EXT_RTC_TIMEOUT = -5
} ext_rtc_return_code;

typedef struct
{
  dev_ctx_t dev_ctx;
  TX_QUEUE *request_queue;
  TX_EVENT_FLAGS_GROUP *complete_flags;
  SPI_HandleTypeDef *rtc_spi_bus;
  gpio_pin_struct int_a_pin;
  gpio_pin_struct int_b_pin;
  gpio_pin_struct ts_1_pin;
  gpio_pin_struct ts_2_pin;
  gpio_pin_struct ts_3_pin;
  gpio_pin_struct ts_4_pin;

  bool ts1_in_use;
  bool ts2_in_use;
  bool ts3_in_use;
  bool ts4_in_use;

  ext_rtc_return_code (*ext_rtc_config_watchdog) ( uint32_t period_ms );
  ext_rtc_return_code (*ext_rtc_refresh_watchdog) ( void );
  ext_rtc_return_code (*ext_rtc_set_date_time) ( struct tm input_date_time );
  ext_rtc_return_code (*ext_rtc_get_date_time) ( struct tm *return_date_time );
  ext_rtc_return_code (*ext_rtc_set_timestamp) ( rtc_timestamp_t which_timestamp );
  ext_rtc_return_code (*ext_rtc_get_timestamp) ( rtc_timestamp_t which_timestamp,
                                                 time_t *return_timestamp );
  ext_rtc_return_code (*ext_rtc_set_alarm) ( rtc_set_alarm_t alarm_setting );
} Ext_RTC;

ext_rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                                   TX_QUEUE *messaging_queue, TX_EVENT_FLAGS_GROUP complete_flags );

#endif /* COMPONENTS_INC_EXT_RTC_H_ */

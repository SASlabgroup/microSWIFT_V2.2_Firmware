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
#define RTC_WATCHDOG_MIN_REFRESH 10000
#define SECONDS_TO_MILLISECONDS(x) (x * 1000)
#define MILLISECONDS_TO_SECONDS(x) (x / 1000)

typedef struct
{
  dev_ctx_t dev_ctx;
  TX_QUEUE *request_queue;
  TX_EVENT_FLAGS_GROUP *complete_flags;
  SPI_HandleTypeDef *rtc_spi_bus;
  gpio_pin_struct int_a_pin;
  gpio_pin_struct int_b_pin;
  gpio_pin_struct ts_pins[NUMBER_OF_TIMESTAMPS];

  bool ts_in_use[NUMBER_OF_TIMESTAMPS];

  uint8_t watchdog_refresh_time_val;

  rtc_return_code (*config_watchdog) ( uint32_t period_ms );
  rtc_return_code (*refresh_watchdog) ( void );
  rtc_return_code (*set_date_time) ( struct tm input_date_time );
  rtc_return_code (*get_date_time) ( struct tm *return_date_time );
  rtc_return_code (*set_timestamp) ( pcf2131_timestamp_t which_timestamp );
  rtc_return_code (*get_timestamp) ( pcf2131_timestamp_t which_timestamp, time_t *return_timestamp );
  rtc_return_code (*set_alarm) ( rtc_alarm_struct alarm_setting );
} Ext_RTC;

rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                               TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags );

#endif /* COMPONENTS_INC_EXT_RTC_H_ */

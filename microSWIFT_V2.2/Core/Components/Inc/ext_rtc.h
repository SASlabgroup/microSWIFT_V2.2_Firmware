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

#define RTC_SPI_TIMEOUT 25
#define RTC_SPI_BUF_SIZE 8

typedef enum
{
  RTC_SUCCESS = 0,
  RTC_SPI_ERROR = -1,
  RTC_PARAMETERS_INVALID = -2
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

  ext_rtc_return_code (*ext_rtc_config_watchdog) ( uint32_t period_ms );
  ext_rtc_return_code (*ext_rtc_refresh_watchdog) ( void );
  ext_rtc_return_code (*ext_rtc_set_date_time) ( struct tm input_date_time );
  ext_rtc_return_code (*ext_rtc_get_date_time) ( struct tm *return_date_time );
  ext_rtc_return_code (*ext_rtc_get_timestamp) ( uint64_t *return_timestamp );
  ext_rtc_return_code (*ext_rtc_set_alarm) ( rtc_alarm_struct alarm_setting );
} Ext_RTC;

ext_rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                                   TX_QUEUE *messaging_queue );

#endif /* COMPONENTS_INC_EXT_RTC_H_ */

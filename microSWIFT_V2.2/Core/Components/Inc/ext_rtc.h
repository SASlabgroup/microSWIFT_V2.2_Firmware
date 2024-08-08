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
} Ext_RTC;

ext_rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                                   TX_QUEUE *messaging_queue );
ext_rtc_return_code ext_rtc_config_watchdog ( uint32_t period_ms );
ext_rtc_return_code ext_rtc_refresh_watchdog ( void );
ext_rtc_return_code ext_rtc_set_date_time ( struct tm input_date_time );
ext_rtc_return_code ext_rtc_get_date_time ( struct tm *return_date_time );
ext_rtc_return_code ext_rtc_get_timestamp ( uint64_t *return_timestamp );
ext_rtc_return_code ext_rtc_set_alarm ( pcf2131_alarm_struct alarm_setting );

#endif /* COMPONENTS_INC_EXT_RTC_H_ */

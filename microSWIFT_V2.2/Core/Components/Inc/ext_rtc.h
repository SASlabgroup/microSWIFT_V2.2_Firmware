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
#include "time.h"
#include "pcf2131_reg.h"

#define RTC_SPI_TIMEOUT 25
#define RTC_SPI_BUF_SIZE 8

typedef enum
{
  RTC_SUCCESS = 0
} ext_rtc_return_codes_t;

typedef enum
{
  RTC_OK = 0,
  RTC_SPI_ERROR = -1
} ext_rtc_return_code;

typedef struct
{
  dev_ctx_t dev_ctx;
  TX_QUEUE *messaging_queue;
  SPI_HandleTypeDef *rtc_spi_bus;
} ext_rtc;

ext_rtc_return_code ext_rtc_init ( ext_rtc *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                                   TX_QUEUE *messaging_queue );
ext_rtc_return_code ext_rtc_config_watchdog ( ext_rtc *struct_ptr, uint32_t period_ms );
ext_rtc_return_code ext_rtc_refresh_watchdog ( ext_rtc *struct_ptr );
ext_rtc_return_code ext_rtc_set_date_time ( ext_rtc *struct_ptr, struct tm input_date_time );
ext_rtc_return_code ext_rtc_get_date_time ( ext_rtc *struct_ptr, struct tm *return_date_time );
ext_rtc_return_code ext_rtc_get_timestamp ( ext_rtc *struct_ptr, uint64_t *return_timestamp );
ext_rtc_return_code ext_rtc_set_alarm ( ext_rtc *struct_ptr, pcf2131_alarm_struct alarm_setting );

#endif /* COMPONENTS_INC_EXT_RTC_H_ */

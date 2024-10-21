/*
 * thread_functions.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef INC_THREADX_SUPPORT_H_
#define INC_THREADX_SUPPORT_H_

#include "stdbool.h"
#include "tx_api.h"
#include "main.h"
#include "gnss.h"
#include "ct_sensor.h"
#include "iridium.h"
#include "temp_sensor.h"
#include "turbidity_sensor.h"
#include "light_sensor.h"
#include "accelerometer_sensor.h"
#include "iridium.h"
#include "stdarg.h"

// @formatter:off

#define ALL_EVENT_FLAGS (0xFFFFFFFF)
#define NO_ERROR_FLAG 0U

typedef enum led_sequence
{
  INITIAL_LED_SEQUENCE      = 1,
  TEST_PASSED_LED_SEQUENCE  = 2,
  TEST_FAILED_LED_SEQUENCE  = 3
} led_sequence_t;

bool        gnss_apply_config ( GNSS *gnss );
bool        ct_self_test ( CT *ct, bool add_warmup_time, ct_sample *self_test_readings );
bool        temperature_self_test ( Temperature *temperature, float *self_test_temp );
bool        turbidity_self_test ( void );
bool        light_self_test ( void );
bool        accelerometer_self_test ( void );
bool        iridium_apply_config ( Iridium *iridium );

void        gnss_error_out(GNSS *gnss, ULONG error_flag, TX_THREAD *gnss_thread, const char *fmt, ...);
void        ct_error_out(CT* ct, ULONG error_flag, TX_THREAD *ct_thread, const char *fmt, ...);
void        temperature_error_out(Temperature* temperature, ULONG error_flag, TX_THREAD *temperature_thread, const char *fmt, ...);
//void        light_error_out (Light* light, ULONG error_flag, TX_THREAD* light_thread, const char *fmt, ...);
//void        turbidity_error_out(Turbidity* turbidity, ULONG error_flag, TX_THREAD *turbidity_thread, const char *fmt, ...);
//void        accelerometer_error_out(Accelerometer* accel, ULONG error_flag, TX_THREAD *accelerometer_thread, const char *fmt, ...);
void        waves_error_out(ULONG error_flag, TX_THREAD *waves_thread, const char *fmt, ...);
void        iridium_error_out(Iridium* iridium, ULONG error_flag, TX_THREAD *iridium_thread, const char *fmt, ...);
void        rtc_error_out(TX_THREAD *rtc_thread, const char *fmt, ...);

bool        is_first_sample_window ( void );

ULONG       get_current_flags ( TX_EVENT_FLAGS_GROUP *event_flags );
void        clear_event_flags ( TX_EVENT_FLAGS_GROUP *event_flags );

uint32_t    ticks_from_milliseconds ( uint32_t milliseconds );

void        led_sequence ( led_sequence_t sequence );

// @formatter:on
#endif /* INC_THREADX_SUPPORT_H_ */

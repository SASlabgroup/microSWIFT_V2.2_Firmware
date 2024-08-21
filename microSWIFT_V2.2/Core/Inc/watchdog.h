/*
 * watchdog.h
 *
 *  Created on: Jun 14, 2024
 *      Author: philbush
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "stdbool.h"
#include "iwdg.h"
#include "tx_api.h"
#include "error_handler.h"
#include "threadx_support.h"

#define WATCHDOG_OK 0
#define WATCHDOG_ERROR 1

enum watchdog_thread_flags
{
  ADC_CHECK_IN = (ULONG) 1 << 0,
  ACC_MAG_CHECK_IN = (ULONG) 1 << 1,
  ACC_GYRO_CHECK_IN = (ULONG) 1 << 2,
  APF11_COMMS_CHECK_IN = (ULONG) 1 << 3,
  FILEX_CHECK_IN = (ULONG) 1 << 4,
  CONTROL_CHECK_IN = (ULONG) 1 << 5
};

struct watchdog_t
{
  IWDG_HandleTypeDef *iwdg_handle;

  TX_EVENT_FLAGS_GROUP *check_in_flags;

  bool adc_shutdown;
  bool acc_mag_shutdown;
  bool acc_gyro_shutdown;
  bool filex_shutdown;
  bool apf11_comms_shutdown;

};

typedef struct watchdog_t *watchdog_handle;

int32_t watchdog_init ( watchdog_handle handle, TX_EVENT_FLAGS_GROUP *watchdog_check_in_flags );
void watchdog_event_callback ( TX_EVENT_FLAGS_GROUP *flags_group_ptr );
void watchdog_check_in ( watchdog_handle wd_handle, enum watchdog_thread_flags thread );
void watchdog_register_error ( watchdog_handle wd_handle, enum thread which_thread );
void watchdog_deregister_error ( watchdog_handle wd_handle, enum thread which_thread );

#endif /* INC_WATCHDOG_H_ */

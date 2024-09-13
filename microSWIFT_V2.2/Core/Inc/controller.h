/*
 * controller.h
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "configuration.h"
#include "NEDWaves/rtwhalf.h"
#include "battery.h"
#include "rf_switch.h"
#include "tx_api.h"

#define STARTUP_SEQUENCE_MAX_WAIT_TICKS (TX_TIMER_TICKS_PER_SECOND * 20)

// @formatter:off
typedef struct
{
  RF_Switch                 rf_switch;
  Battery                   battery;

  microSWIFT_configuration  *global_config;

  Thread_Handles            *thread_handles;

  TX_EVENT_FLAGS_GROUP      *error_flags;
  TX_EVENT_FLAGS_GROUP      *init_flags;
  TX_EVENT_FLAGS_GROUP      *irq_flags;
  TX_EVENT_FLAGS_GROUP      *complete_flags;
  TX_TIMER                  *timer;

  bool                      (*startup_procedure)( void );
  bool                      (*all_threads_complete) ( void );
  real16_T                  (*get_battery_voltage) ( void );
  void                      (*shutdown_all_pheripherals) ( void );
  void                      (*enter_processor_shutdown_mode) ( void );
  void                      (*monitor_and_handle_errors) ( void );
} Control;
// @formatter:on

void controller_init ( Control *struct_ptr, microSWIFT_configuration *global_config,
                       Thread_Handles *thread_handles, TX_EVENT_FLAGS_GROUP *error_flags,
                       TX_EVENT_FLAGS_GROUP *init_flags, TX_EVENT_FLAGS_GROUP *irq_flags,
                       TX_EVENT_FLAGS_GROUP *complete_flags, TX_TIMER *timer,
                       ADC_HandleTypeDef *battery_adc_handle );

void control_timer_expired ( ULONG expiration_input );

#endif /* INC_CONTROLLER_H_ */

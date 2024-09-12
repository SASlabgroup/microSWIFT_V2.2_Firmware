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

// @formatter:off
typedef struct
{
  RF_Switch                 rf_switch;
  Battery                   battery;

  microSWIFT_configuration  *global_config;

  Thread_Handles            *thread_handles;

  TX_EVENT_FLAGS_GROUP      *error_flags;
  TX_EVENT_FLAGS_GROUP      *init_flags;
  TX_EVENT_FLAGS_GROUP      *complete_flags;
  TX_TIMER                  *timer;

  bool                      (*startup_procedure)( void );
  real16_T                  (*get_battery_voltage) ( void );
  void                      (*set_rf_switch_port) (rf_switch_selection_t port);
  void                      (*shutdown) ( void );
} Control;
// @formatter:on

void controller_init ( Control *struct_ptr, microSWIFT_configuration *global_config,
                       Thread_Handles *thread_handles, TX_EVENT_FLAGS_GROUP *error_flags,
                       TX_EVENT_FLAGS_GROUP *init_flags, TX_EVENT_FLAGS_GROUP *complete_flags,
                       TX_TIMER *timer );

void control_timer_expired ( ULONG expiration_input );

#endif /* INC_CONTROLLER_H_ */

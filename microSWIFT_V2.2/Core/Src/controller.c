/*
 * controller.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include "controller.h"

// @formatter:off
static Control *self;

static bool     _control_startup_procedure(void);
static real16_T _control_get_battery_voltage(void);
static void     _control_set_rf_switch_port(rf_switch_selection_t port);
static void     _control_shutdown(void);
// @formatter:on

void controller_init ( Control *struct_ptr, microSWIFT_configuration *global_config,
                       Thread_Handles *thread_handles, TX_EVENT_FLAGS_GROUP *error_flags,
                       TX_EVENT_FLAGS_GROUP *init_flags, TX_EVENT_FLAGS_GROUP *complete_flags,
                       TX_TIMER *timer )
{
  self = struct_ptr;

  self->global_config = global_config;
  self->thread_handles = thread_handles;
  self->error_flags = error_flags;
  self->init_flags = init_flags;
  self->complete_flags = complete_flags;
  self->timer = timer;

  self->startup_procedure = _control_startup_procedure;
  self->get_battery_voltage = _control_get_battery_voltage;
  self->set_rf_switch_port = _control_set_rf_switch_port;
  self->shutdown = _control_shutdown;
}

static bool _control_startup_procedure ( void )
{

}

static real16_T _control_get_battery_voltage ( void );
static void _control_set_rf_switch_port ( rf_switch_selection_t port );
static void _control_shutdown ( void );

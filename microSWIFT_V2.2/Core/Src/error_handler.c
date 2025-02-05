/*
 * error_handler.c
 *
 *  Created on: Aug 22, 2024
 *      Author: philbush
 */

#include "error_handler.h"
#include "threadx_support.h"
#include "persistent_ram.h"
#include "app_threadx.h"
#include "main.h"

void Error_Handler ( void )
{
  __disable_irq ();

  safe_mode ();

  persistent_ram_deinit ();

  HAL_NVIC_SystemReset ();

  __enable_irq ();
}

void safe_mode ( void )
{
#warning "Turn off all peripherals here, then wait a short amount of time"
}

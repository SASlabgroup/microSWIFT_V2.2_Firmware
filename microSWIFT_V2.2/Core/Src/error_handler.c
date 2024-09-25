/*
 * error_handler.c
 *
 *  Created on: Aug 22, 2024
 *      Author: philbush
 */

#include "error_handler.h"

#warning "ensure error handler de-registers thread on error"

void Error_Handler ( void )
{
  __asm__("BKPT");
}

void safe_mode ( void );

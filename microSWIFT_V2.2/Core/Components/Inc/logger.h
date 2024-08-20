/*
 * logger.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LOGGER_H_
#define COMPONENTS_INC_LOGGER_H_

#include "usart.h"

typedef struct
{
  uint32_t placeholder;
} uart_logger;

void uart_logger_init ( void );
void uart_logger_log_line ( const char *line );

#endif /* COMPONENTS_INC_LOGGER_H_ */

/*
 * logger.h
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_LOGGER_H_
#define COMPONENTS_INC_LOGGER_H_

#include "usart.h"
#include "tx_api.h"
#include "basic_stack.h"
#include "stdarg.h"

#define LOG_QUEUE_LENGTH 16

typedef struct
{
  char line_buf[256];
} log_line_buf;

typedef struct
{
  log_line_buf *str_buf;
  size_t strlen;
} logger_message;

typedef struct
{
  // GPIO pin to indicate enable/ disable
  gpio_pin_struct enable_pin;
  // Circular queue for char buffers
  basic_stack_handle buffer_stack;
  // Message queue
  TX_QUEUE *msg_que;
  // UART handle
  UART_HandleTypeDef *uart;
  void (*send_log_line) ( void );
} uart_logger;

void uart_logger_init ( uart_logger logger, basic_stack_handle buffer_Stack, uint8_t *stack_mem,
                        TX_QUEUE *msg_que, UART_HandleTypeDef *uart_handle );

#endif /* COMPONENTS_INC_LOGGER_H_ */

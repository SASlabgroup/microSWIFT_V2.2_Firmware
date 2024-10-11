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
#include "stdbool.h"
#include "gpio.h"
#include "time.h"

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
  // Block pool for the char buffers
  TX_BLOCK_POOL *block_pool;
  // Message queue
  TX_QUEUE *msg_que;
  // UART handle
  UART_HandleTypeDef *uart;
  void (*send_log_line) ( log_line_buf *buf, size_t strlen );
  void (*return_line_buffer) ( log_line_buf *buffer );
  bool logger_enabled;
} uart_logger;

void uart_logger_init ( uart_logger *logger, TX_BLOCK_POOL *block_pool, TX_QUEUE *msg_que,
                        UART_HandleTypeDef *uart_handle );
void uart_logger_log_line ( const char *fmt, ... );

#endif /* COMPONENTS_INC_LOGGER_H_ */

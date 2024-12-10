/*
 * logger.c
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#include <ext_rtc_server.h>
#include "logger.h"
#include "usart.h"
#include "tx_api.h"
#include "basic_stack.h"
#include "stdarg.h"
#include "time.h"
#include "string.h"
#include "stdio.h"

static uart_logger *logger_self;

const char *rtc_err_str = "RTC ERROR";

static UINT _logger_get_buffer ( log_line_buf **line_buf );
static void _logger_send_log_line ( log_line_buf *buf, size_t strlen );
static void _logger_return_buffer ( log_line_buf *buffer );

static void __reset_uart ( void );

void uart_logger_init ( uart_logger *logger, TX_BLOCK_POOL *block_pool, TX_QUEUE *msg_que,
                        TX_MUTEX *mutex, UART_HandleTypeDef *uart_handle )
{
  logger_self = logger;

  logger_self->enable_pin.port = UART_LOGGER_EN_GPIO_Port;
  logger_self->enable_pin.pin = UART_LOGGER_EN_Pin;

  logger_self->block_pool = block_pool;
  logger_self->msg_que = msg_que;
  logger_self->lock = mutex;
  logger_self->uart = uart_handle;
  logger_self->send_log_line = _logger_send_log_line;
  logger_self->return_line_buffer = _logger_return_buffer;

  // If the enable pin is being pulled to ground, enable the logger
  if ( HAL_GPIO_ReadPin (logger_self->enable_pin.port, logger_self->enable_pin.pin)
       == GPIO_PIN_RESET )
  {
    logger_self->logger_enabled = usart3_init () == UART_OK;
  }
  else
  {
    logger_self->logger_enabled = false;
  }
}

void uart_log ( const char *fmt, ... )
{
  if ( tx_mutex_get (logger_self->lock, MUTEX_LOCK_TICKS) != TX_SUCCESS )
  {
    return;
  }

  log_line_buf *log_buf = TX_NULL;
  logger_message msg;
  size_t str_len = 0;
  size_t bytes_remaining = sizeof(log_line_buf);
  va_list args;
  va_start(args, fmt);

  UINT tx_ret;

  tx_ret = _logger_get_buffer (&log_buf);

  if ( tx_ret != TX_SUCCESS )
  {
    va_end(args);
    return;
  }

  memset (log_buf, 0, sizeof(log_line_buf));

  str_len = vsnprintf (&(log_buf->line_buf[sizeof(log_line_buf) - bytes_remaining]),
                       bytes_remaining, fmt, args);

  va_end(args);

  bytes_remaining -= str_len + 1;

  msg.str_buf = log_buf;
  msg.strlen = sizeof(log_line_buf) - bytes_remaining;

  // Put a break line at the end
  strcat (&log_buf->line_buf[0], "\n");

  (void) tx_queue_send (logger_self->msg_que, (VOID*) &msg, TX_NO_WAIT);

  (void) tx_mutex_put (logger_self->lock);
}

static UINT _logger_get_buffer ( log_line_buf **line_buf )
{
  return tx_block_allocate (logger_self->block_pool, (VOID**) line_buf, TX_NO_WAIT);
}

static void _logger_send_log_line ( log_line_buf *buf, size_t strlen )
{
  if ( logger_self->logger_enabled )
  {
    HAL_UART_Transmit_DMA (logger_self->uart, (uint8_t*) &(buf->line_buf[0]), strlen);
  }
}

static void _logger_return_buffer ( log_line_buf *buffer )
{
  (void) tx_block_release ((VOID*) buffer);
}

static void __reset_uart ( void )
{
  usart6_deinit ();
  usart6_init ();
}

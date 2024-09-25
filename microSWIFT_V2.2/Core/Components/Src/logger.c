/*
 * logger.c
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#include "logger.h"
#include "usart.h"
#include "tx_api.h"
#include "basic_stack.h"
#include "stdarg.h"
#include "time.h"
#include "string.h"
#include "stdio.h"
#include "ext_rtc_api.h"

static uart_logger *logger_self;

const char *rtc_err_str = "RTC ERROR";

static UINT _logger_get_buffer ( log_line_buf *line_buf );
static void _logger_send_log_line ( log_line_buf *buf, size_t strlen );
static void _logger_return_buffer ( log_line_buf *buffer );

void uart_logger_init ( uart_logger *logger, TX_BLOCK_POOL *block_pool, TX_QUEUE *msg_que,
                        UART_HandleTypeDef *uart_handle )
{
  logger_self = logger;

  logger_self->enable_pin.port = UART_LOGGER_EN_GPIO_Port;
  logger_self->enable_pin.pin = UART_LOGGER_EN_Pin;

  logger_self->block_pool = block_pool;
  logger_self->msg_que = msg_que;
  logger_self->uart = uart_handle;
  logger_self->send_log_line = _logger_send_log_line;
  logger_self->return_line_buffer = _logger_return_buffer;

  if ( HAL_GPIO_ReadPin (logger_self->enable_pin.port, logger_self->enable_pin.pin)
       == GPIO_PIN_SET )
  {
    logger_self->logger_enabled = true;
  }
  else
  {
    logger_self->logger_enabled = false;
  }
}

void uart_logger_log_line ( const char *fmt, ... )
{
  log_line_buf *log_buf = NULL;
  logger_message msg;
  size_t str_len = 0;
  size_t bytes_remaining = sizeof(log_line_buf);
  struct tm time;
  va_list args;
  va_start(args, fmt);
  UINT tx_ret;

  tx_ret = _logger_get_buffer (log_buf);

  if ( (tx_ret != TX_SUCCESS) || !logger_self->logger_enabled )
  {
    va_end(args);
    return;
  }

  if ( rtc_server_get_time (&time, LOGGER_REQUEST_PROCESSED) == RTC_SUCCESS )
  {
    str_len = strftime (&(log_buf->line_buf[0]), bytes_remaining, "%D %T: ", &time);
    bytes_remaining -= str_len;
  }
  else
  {
    str_len = strlen (rtc_err_str);
    strncpy (&(log_buf->line_buf[0]), rtc_err_str, str_len);
    bytes_remaining -= str_len;
  }

  str_len = snprintf (&(log_buf->line_buf[sizeof(log_line_buf) - bytes_remaining]), strlen (fmt),
                      fmt, args);
  bytes_remaining -= str_len;

  va_end(args);

  msg.str_buf = log_buf;
  msg.strlen = sizeof(log_line_buf) - bytes_remaining;

  (void) tx_queue_send (logger_self->msg_que, (VOID*) &msg, TX_NO_WAIT);
}

static UINT _logger_get_buffer ( log_line_buf *line_buf )
{
  return tx_block_allocate (logger_self->block_pool, (VOID**) &line_buf, TX_NO_WAIT);
}

static void _logger_send_log_line ( log_line_buf *buf, size_t strlen )
{
  HAL_UART_Transmit_DMA (logger_self->uart, (uint8_t*) &(buf->line_buf[0]), strlen);
}

static void _logger_return_buffer ( log_line_buf *buffer )
{
  (void) tx_block_release ((VOID*) buffer);
}


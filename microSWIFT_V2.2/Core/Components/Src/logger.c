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

static uart_logger *self;

const char *rtc_err_str = "RTC ERROR";

static bool uart_logger_get_buffer ( uart_logger *logger, log_line_buf *line_buf );
static void _logger_send_log_line ( log_line_buf *buf, size_t strlen );
static void _logger_return_buffer ( log_line_buf *buffer );

void uart_logger_init ( uart_logger *logger, basic_stack_handle buffer_stack, uint8_t *stack_mem,
                        TX_QUEUE *msg_que, UART_HandleTypeDef *uart_handle )
{
  self = logger;

  self->enable_pin.port = UART_LOGGER_EN_GPIO_Port;
  self->enable_pin.pin = UART_LOGGER_EN_Pin;

  self->buffer_stack = buffer_stack;
  self->msg_que = msg_que;
  self->uart = uart_handle;
  self->send_log_line = _logger_send_log_line;
  self->return_line_buffer = _logger_return_buffer;

  if ( HAL_GPIO_ReadPin (self->enable_pin.port, self->enable_pin.pin) == GPIO_PIN_SET )
  {
    self->logger_enabled = true;
  }
  else
  {
    self->logger_enabled = false;
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

  bool stack_empty = uart_logger_get_buffer (self, log_buf);

  if ( stack_empty || !self->logger_enabled )
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

  (void) tx_queue_send (self->msg_que, (VOID*) &msg, TX_NO_WAIT);
}

static bool uart_logger_get_buffer ( uart_logger *logger, log_line_buf *line_buf )
{
  return bs_pop_element (logger->buffer_stack, (void*) line_buf);
}

static void _logger_send_log_line ( log_line_buf *buf, size_t strlen )
{
  HAL_UART_Transmit_DMA (self->uart, (uint8_t*) &(buf->line_buf[0]), strlen);
}

static void _logger_return_buffer ( log_line_buf *buffer )
{
  memset (&(buffer->line_buf[0]), 0, sizeof(log_line_buf));
  bs_push_element (self->buffer_stack, (void*) buffer);
}


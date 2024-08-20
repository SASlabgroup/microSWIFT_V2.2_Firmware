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
#include "ext_rtc_api.h"

uart_logger *self;

const char *rtc_err_str = "RTC ERROR";

static bool uart_logger_get_buffer ( uart_logger *logger, log_line_buffer *line_buf );
static void _send_log_line ( void );

void uart_logger_init ( uart_logger logger, basic_stack_handle buffer_Stack, uint8_t *stack_mem,
                        TX_QUEUE *msg_que, UART_HandleTypeDef *uart_handle )
{
  self = logger;

//  self->enable_pin.port = 

}

void uart_logger_log_line ( const char *fmt, ... )
{
  log_line_buffer *log_buf;
  logger_message msg;
  size_t str_len = 0;
  size_t bytes_remaining = sizeof(log_line_buffer);
  struct tm time;
  va_list args;
  va_start(args, fmt);

  bool stack_empty = uart_logger_get_buffer (self, log_buf);
  if ( !stack_empty )
  {
    if ( rtc_server_get_time (&time, LOGGER_REQUEST_PROCESSED) == RTC_SUCCESS )
    {
      str_len = strftime (&(log_buf->line_buf[0]), bytes_remaining, "%D% %T", &time);
      bytes_remaining -= str_len;
    }
    else
    {
      str_len = strlen (rtc_err_str);
      strncpy (&(line_buf[0]), rtc_err_str, str_len);
      bytes_remaining -= str_len;
    }

    str_len = snprintf (&(log_buf->line_buf[sizeof(log_line_buffer) - bytes_remaining]), fmt, args);
    bytes_remaining -= str_len;
    va_end(args);
  }

  msg.str_buf = log_buf;
  msg.strlen = sizeof(log_line_buffer) - bytes_remaining;

  (void) tx_queue_send (self->msg_queue, (VOID*) &msg, TX_NO_WAIT);
}

static bool uart_logger_get_buffer ( uart_logger *logger, log_line_buffer *line_buf )
{
  return bs_pop_element (logger->buffer_stack, (void*) line_buf);
}

"Something happened %d", thing.element

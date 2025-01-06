/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include <ext_rtc_server.h>
#include "iridium.h"
#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "NEDWaves/rtwhalf.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_tim.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"
#include "time.h"
#include "configuration.h"
#include "usart.h"
#include "persistent_ram.h"
#include "watchdog.h"
#include "logger.h"
#include "stdarg.h"

// @formatter:off
// Object pointer
static Iridium *iridium_self;
// Object functions
static uSWIFT_return_code_t  _iridium_config ( void );
static uSWIFT_return_code_t  _iridium_self_test ( void );
static uSWIFT_return_code_t  _iridium_start_timer ( uint16_t timeout_in_minutes );
static uSWIFT_return_code_t  _iridium_stop_timer ( void );
static uSWIFT_return_code_t  _iridium_transmit_message ( uint8_t *msg, uint32_t msg_size );
static void                  _iridium_charge_caps ( uint32_t caps_charge_time_ticks );
static void                  _iridium_sleep ( void );
static void                  _iridium_wake ( void );
static void                  _iridium_on ( void );
static void                  _iridium_off ( void );

// Helper functions
static uSWIFT_return_code_t  __send_basic_command_message ( const char *command, uint8_t response_size );
static uSWIFT_return_code_t  __internal_transmit_message ( uint8_t *payload, uint16_t payload_size );
static void                  __cycle_power ( void );
static void                  __get_checksum ( uint8_t *payload, size_t payload_size );
static int32_t               __uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size, uint32_t timeout_ticks );
static int32_t               __uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size, uint32_t timeout_ticks );

// AT commands
static const char *ack                          = "AT\r";
static const char *disable_flow_control         = "AT&K0\r";
static const char *enable_ring_indications      = "AT+SBDMTA=1\r";
static const char *store_config                 = "AT&W0\r";
static const char *select_power_up_profile      = "AT&Y0\r";
static const char *clear_MO                     = "AT+SBDD0\r";
static const char *send_sbd                     = "AT+SBDIX\r";
// @formatter:on

/**
 * Initialize the Iridium struct
 *
 * @return void
 */
void iridium_init ( Iridium *struct_ptr, microSWIFT_configuration *global_config,
                    UART_HandleTypeDef *iridium_uart_handle, TX_SEMAPHORE *uart_sema,
                    DMA_HandleTypeDef *uart_tx_dma_handle, DMA_HandleTypeDef *uart_rx_dma_handle,
                    TX_TIMER *timer, TX_EVENT_FLAGS_GROUP *error_flags )
{
  // Assign the object pointer
  iridium_self = struct_ptr;

  iridium_self->global_config = global_config;
  iridium_self->uart_tx_dma_handle = uart_tx_dma_handle;
  iridium_self->uart_rx_dma_handle = uart_rx_dma_handle;
  iridium_self->timer = timer;
  iridium_self->error_flags = error_flags;

  memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

  iridium_self->bus_5v_fet.port = BUS_5V_FET_GPIO_Port;
  iridium_self->bus_5v_fet.pin = BUS_5V_FET_Pin;
  iridium_self->pwr_pin.port = IRIDIUM_FET_GPIO_Port;
  iridium_self->pwr_pin.pin = IRIDIUM_FET_Pin;
  iridium_self->sleep_pin.port = IRIDIUM_OnOff_GPIO_Port;
  iridium_self->sleep_pin.pin = IRIDIUM_OnOff_Pin;

  iridium_self->timer_timeout = false;

  iridium_self->config = _iridium_config;
  iridium_self->self_test = _iridium_self_test;
  iridium_self->transmit_message = _iridium_transmit_message;
  iridium_self->start_timer = _iridium_start_timer;
  iridium_self->stop_timer = _iridium_stop_timer;
  iridium_self->charge_caps = _iridium_charge_caps;
  iridium_self->sleep = _iridium_sleep;
  iridium_self->wake = _iridium_wake;
  iridium_self->on = _iridium_on;
  iridium_self->off = _iridium_off;

  generic_uart_register_io_functions (&iridium_self->uart_driver, iridium_uart_handle, uart_sema,
                                      uart4_init, uart4_deinit, __uart_read, __uart_write);
}

/**
 * Deinit the UART and DMA Tx/ Rx channels
 *
 * @return void
 */
void iridium_deinit ( void )
{
  (void) iridium_self->uart_driver.deinit ();
  HAL_DMA_DeInit (iridium_self->uart_rx_dma_handle);
  HAL_DMA_DeInit (iridium_self->uart_tx_dma_handle);
}

/**
 *
 * @return void
 */
void iridium_timer_expired ( ULONG expiration_input )
{
  (void) expiration_input;
  iridium_self->timer_timeout = true;
}

/**
 *
 *
 * @return bool
 */
bool iridium_get_timeout_status ( void )
{
  return iridium_self->timer_timeout;
}

/**
 *
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t _iridium_config ( void )
{
  uSWIFT_return_code_t return_code;

  // Get an ack message
  return_code = __send_basic_command_message (ack, ACK_MESSAGE_SIZE);
  if ( return_code != uSWIFT_SUCCESS )
  {
    return return_code;
  }
  // disable flow control
  return_code = __send_basic_command_message (disable_flow_control, DISABLE_FLOW_CTRL_SIZE);
  if ( return_code != uSWIFT_SUCCESS )
  {
    return return_code;
  }
  // enable SBD ring indications
  return_code = __send_basic_command_message (enable_ring_indications, ENABLE_RI_SIZE);
  if ( return_code != uSWIFT_SUCCESS )
  {
    return return_code;
  }
  // Store this configuration as profile 0
  return_code = __send_basic_command_message (store_config, STORE_CONFIG_SIZE);
  if ( return_code != uSWIFT_SUCCESS )
  {
    return return_code;
  }
  // set profile 0 as the power-up profile
  return_code = __send_basic_command_message (select_power_up_profile, SELECT_PWR_UP_SIZE);
  if ( return_code != uSWIFT_SUCCESS )
  {
    return return_code;
  }

  return return_code;
}

/**
 *
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t _iridium_self_test ( void )
{
  return __send_basic_command_message (ack, ACK_MESSAGE_SIZE);
}

/**
 *
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t _iridium_start_timer ( uint16_t timeout_in_minutes )
{
  ULONG timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( tx_timer_change (iridium_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (iridium_self->timer) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMER_ERROR;
  }

  return ret;
}

/**
 *
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t _iridium_stop_timer ( void )
{
  return (tx_timer_deactivate (iridium_self->timer) == TX_SUCCESS) ?
      uSWIFT_SUCCESS : uSWIFT_TIMER_ERROR;
}

/**
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t _iridium_transmit_message ( uint8_t *msg, uint32_t msg_size )
{
  uSWIFT_return_code_t return_code = uSWIFT_SUCCESS;
  uSWIFT_return_code_t queue_return_code __attribute__((unused));

  watchdog_check_in (IRIDIUM_THREAD);

  return_code = __internal_transmit_message (msg, msg_size - IRIDIUM_CHECKSUM_LENGTH);

  if ( return_code == uSWIFT_IO_ERROR )
  {
    __cycle_power ();
    return_code = iridium_self->config ();
  }

  return return_code;
}

/**
 *
 * @return void
 */
static void _iridium_charge_caps ( uint32_t caps_charge_time_ticks )
{
  // Power the unit by pulling the sleep pin to ground.
  iridium_self->on ();
  iridium_self->wake ();

  tx_thread_sleep (caps_charge_time_ticks);
}

/**
 *
 * @return void
 */
static void _iridium_sleep ( void )
{
  HAL_GPIO_WritePin (iridium_self->sleep_pin.port, iridium_self->sleep_pin.pin, GPIO_PIN_RESET);
}

/**
 *
 * @return void
 */
static void _iridium_wake ( void )
{
  HAL_GPIO_WritePin (iridium_self->sleep_pin.port, iridium_self->sleep_pin.pin, GPIO_PIN_SET);
}

/**
 *
 * @return void
 */
static void _iridium_on ( void )
{
  HAL_GPIO_WritePin (iridium_self->bus_5v_fet.port, iridium_self->bus_5v_fet.pin, GPIO_PIN_SET);
  tx_thread_sleep (1);
  HAL_GPIO_WritePin (iridium_self->pwr_pin.port, iridium_self->pwr_pin.pin, GPIO_PIN_SET);
}

/**
 *
 * @return void
 */
static void _iridium_off ( void )
{
  HAL_GPIO_WritePin (iridium_self->pwr_pin.port, iridium_self->pwr_pin.pin, GPIO_PIN_RESET);
  tx_thread_sleep (1);
  HAL_GPIO_WritePin (iridium_self->bus_5v_fet.port, iridium_self->bus_5v_fet.pin, GPIO_PIN_RESET);
}

/**
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t __send_basic_command_message ( const char *command,
                                                           uint8_t response_size )
{
  char *needle;
  ULONG actual_flags;

  memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

  if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(command[0]),
                                        strlen (command), IRIDIUM_MAX_UART_TX_TICKS)
       != UART_OK )
  {
    return uSWIFT_IO_ERROR;
  }

  if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                       &(iridium_self->response_buffer[0]), response_size,
                                       IRIDIUM_MAX_UART_RX_TICKS_NO_TX)
       != UART_OK )
  {
    return uSWIFT_IO_ERROR;
  }

  needle = strstr ((char*) &(iridium_self->response_buffer[1]), "OK");
  memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
  return (needle == NULL) ?
      uSWIFT_IO_ERROR : uSWIFT_SUCCESS;
}

/**
 *
 * @return uSWIFT_return_code_t
 */
static uSWIFT_return_code_t __internal_transmit_message ( uint8_t *payload, uint16_t payload_size )
{
  uSWIFT_return_code_t return_code = uSWIFT_TIMEOUT;
  ULONG actual_flags;
  char *needle;
  char *sbdix_search_term = "+SBDIX: ";
  char payload_size_str[4];
  char load_sbd[15] = "AT+SBDWB=";
  char SBDWB_response_code;
  int SBDIX_response_code, fail_counter, tx_response_time, max_retries = 10;
  bool checksum_match;
  bool message_response_received;

  // Assemble the load_sbd string
  itoa (payload_size, payload_size_str, 10);
  strcat (load_sbd, payload_size_str);
  strcat (load_sbd, "\r");

  while ( !iridium_self->timer_timeout )
  {

    message_response_received = false;
    tx_response_time = 0;

    // get the checksum
    __get_checksum ((uint8_t*) payload, payload_size);

    // Tell the modem we want to send a message
    for ( fail_counter = 0; fail_counter < max_retries; fail_counter++ )
    {
      watchdog_check_in (IRIDIUM_THREAD);

      if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(load_sbd[0]),
                                            strlen (load_sbd), IRIDIUM_MAX_UART_TX_TICKS)
           != UART_OK )
      {
        return uSWIFT_IO_ERROR;
      }

      if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                           &(iridium_self->response_buffer[0]),
                                           SBDWB_READY_RESPONSE_SIZE,
                                           IRIDIUM_MAX_UART_RX_TICKS_NO_TX)
           != UART_OK )
      {
        return uSWIFT_IO_ERROR;
      }

      needle = strstr ((char*) &(iridium_self->response_buffer[0]), "READY");

      // Clear the response buffer and reset UART for the next step
      memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

      // Success case
      if ( needle != NULL )
      {
        break;
      }
    }

    if ( fail_counter == max_retries )
    {
      return_code = uSWIFT_IO_ERROR;
      continue;
    }

    // Send over the payload + checksum
    for ( fail_counter = 0; fail_counter < max_retries; fail_counter++ )
    {
      watchdog_check_in (IRIDIUM_THREAD);

      if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(payload[0]),
                                            payload_size + IRIDIUM_CHECKSUM_LENGTH,
                                            IRIDIUM_MAX_UART_TX_TICKS)
           != UART_OK )
      {
        return uSWIFT_IO_ERROR;
      }

      if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                           &(iridium_self->response_buffer[0]),
                                           SBDWB_LOAD_RESPONSE_SIZE,
                                           IRIDIUM_MAX_UART_RX_TICKS_NO_TX)
           != UART_OK )
      {
        return uSWIFT_IO_ERROR;
      }

      SBDWB_response_code = iridium_self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];

      memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

      // Success case
      if ( SBDWB_response_code == '0' )
      {
        checksum_match = true;
        break;
      }

      // Response of 2 means checksum didn't match, loop around and try again
      if ( SBDWB_response_code == '2' )
      {
        checksum_match = false;
        break;
      }
    }

    if ( (fail_counter == max_retries) || (!checksum_match) )
    {
      return_code = uSWIFT_IO_ERROR;
      continue;
    }

    watchdog_check_in (IRIDIUM_THREAD);

    // Tell the modem to send the message
    if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(send_sbd[0]),
                                          strlen (send_sbd), IRIDIUM_MAX_UART_TX_TICKS)
         != UART_OK )
    {
      return uSWIFT_IO_ERROR;
    }

    if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                         &(iridium_self->response_buffer[0]),
                                         SBDIX_RESPONSE_SIZE,
                                         IRIDIUM_MAX_UART_RX_TICKS_TX)
         != UART_OK )
    {
      return uSWIFT_IO_ERROR;
    }

    watchdog_check_in (IRIDIUM_THREAD);

    // Grab the MO status
    needle = strstr ((char*) &(iridium_self->response_buffer[0]), sbdix_search_term);
    needle += strlen (sbdix_search_term);
    SBDIX_response_code = atoi (needle);

    if ( SBDIX_response_code <= 4 )
    {
      // Success case
      __send_basic_command_message (clear_MO, SBDD_RESPONSE_SIZE);
      LOG("Iridium transmission successful.");
      return uSWIFT_SUCCESS;
    }

    LOG("Iridium transmission unsuccessful. MO status: %d", SBDIX_response_code);

    // If message Tx failed, put the modem to sleep and delay for a total of 30 seconds
    iridium_self->sleep ();

    if ( iridium_self->timer_timeout )
    {
      break;
    }
    // Make sure we've got enough time left to try again
    else if ( get_timer_remaining_ticks (iridium_self->timer)
              < (MODEM_SLEEP_TIME + IRIDIUM_TOP_UP_CAP_CHARGE_TIME + (TX_TIMER_TICKS_PER_SECOND * 5)) )
    {
      break;
    }

    watchdog_check_in (IRIDIUM_THREAD);
    tx_thread_sleep (MODEM_SLEEP_TIME);
    watchdog_check_in (IRIDIUM_THREAD);

    if ( iridium_self->timer_timeout )
    {
      break;
    }

    iridium_self->wake ();
    iridium_self->charge_caps (IRIDIUM_TOP_UP_CAP_CHARGE_TIME);
    watchdog_check_in (IRIDIUM_THREAD);

    memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
    return_code = uSWIFT_TIMEOUT;
  }

  watchdog_check_in (IRIDIUM_THREAD);
  return return_code;
}

/**
 *
 * @return void
 */
static void __cycle_power ( void )
{
  iridium_self->off ();
  iridium_self->sleep ();
  tx_thread_sleep (3);
  iridium_self->on ();
  iridium_self->wake ();
  tx_thread_sleep (3);
}

/**
 *
 * @return void
 */
static void __get_checksum ( uint8_t *payload, size_t payload_size )
{
#warning "This should be redone to return a checksum, not directly append it to a buffer."

  uint16_t checksum = 0;
  uint8_t *checksum_ptr = (uint8_t*) &checksum;
  // calculate checksum
  for ( int i = 0; i < payload_size; i++ )
  {
    checksum += payload[i];
  }
  // place checksum in the last two bytes of the payload array
  payload[payload_size + 1] = ((uint8_t) *checksum_ptr);
  checksum_ptr++;
  payload[payload_size] = ((uint8_t) *checksum_ptr);
}

static int32_t __uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size,
                             uint32_t timeout_ticks )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Receive_DMA (driver_handle->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    HAL_UART_DMAStop (driver_handle->uart_handle);
    HAL_UART_Abort (driver_handle->uart_handle);
    return UART_ERR;
  }

  return UART_OK;
}

static int32_t __uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size,
                              uint32_t timeout_ticks )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Transmit_DMA (driver_handle->uart_handle, write_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    HAL_UART_DMAStop (driver_handle->uart_handle);
    HAL_UART_Abort (driver_handle->uart_handle);
    return UART_ERR;
  }

  return UART_OK;
}


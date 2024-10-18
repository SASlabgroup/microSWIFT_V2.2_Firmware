/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

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
#include "ext_rtc_api.h"
#include "usart.h"
#include "persistent_ram.h"
#include "watchdog.h"
#include "logger.h"
#include "stdarg.h"

// @formatter:off
// Object pointer
static Iridium *iridium_self;
// Object functions
static iridium_return_code_t _iridium_config ( void );
static iridium_return_code_t _iridium_self_test ( void );
static iridium_return_code_t _iridium_start_timer ( uint16_t timeout_in_minutes );
static iridium_return_code_t _iridium_stop_timer ( void );
static iridium_return_code_t _iridium_transmit_message ( sbd_message_type_52 *msg );
static iridium_return_code_t _iridium_transmit_error_message ( char *error_message );
static void                  _iridium_log_error_string (char* str, ...);
static void                  _iridium_charge_caps ( uint32_t caps_charge_time_ticks );
static void                  _iridium_sleep ( void );
static void                  _iridium_wake ( void );
static void                  _iridium_on ( void );
static void                  _iridium_off ( void );

// Helper functions
static iridium_return_code_t __send_basic_command_message ( const char *command, uint8_t response_size );
static iridium_return_code_t __internal_transmit_message ( uint8_t *payload, uint16_t payload_size );
static void                  __cycle_power ( void );
static void                  __get_checksum ( uint8_t *payload, size_t payload_size );
static int32_t               __uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size );
static int32_t               __uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size );

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
  iridium_self->transmit_error_message = _iridium_transmit_error_message;
  iridium_self->start_timer = _iridium_start_timer;
  iridium_self->stop_timer = _iridium_stop_timer;
  iridium_self->charge_caps = _iridium_charge_caps;
  iridium_self->sleep = _iridium_sleep;
  iridium_self->wake = _iridium_wake;
  iridium_self->on = _iridium_on;
  iridium_self->off = _iridium_off;

  generic_uart_register_io_functions (&iridium_self->uart_driver, iridium_uart_handle, uart_sema,
                                      uart4_init, uart4_deinit, __uart_read, __uart_write);
  generic_uart_set_timeout_ticks (&iridium_self->uart_driver, IRIDIUM_MAX_UART_TX_TICKS,
  IRIDIUM_MAX_UART_RX_TICKS_NO_TX);
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
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_config ( void )
{
  iridium_return_code_t return_code;

  // Get an ack message
  return_code = __send_basic_command_message (ack, ACK_MESSAGE_SIZE);
  if ( return_code != IRIDIUM_SUCCESS )
  {
    return return_code;
  }
  // disable flow control
  return_code = __send_basic_command_message (disable_flow_control, DISABLE_FLOW_CTRL_SIZE);
  if ( return_code != IRIDIUM_SUCCESS )
  {
    return return_code;
  }
  // enable SBD ring indications
  return_code = __send_basic_command_message (enable_ring_indications, ENABLE_RI_SIZE);
  if ( return_code != IRIDIUM_SUCCESS )
  {
    return return_code;
  }
  // Store this configuration as profile 0
  return_code = __send_basic_command_message (store_config, STORE_CONFIG_SIZE);
  if ( return_code != IRIDIUM_SUCCESS )
  {
    return return_code;
  }
  // set profile 0 as the power-up profile
  return_code = __send_basic_command_message (select_power_up_profile, SELECT_PWR_UP_SIZE);
  if ( return_code != IRIDIUM_SUCCESS )
  {
    return return_code;
  }

  return return_code;
}

/**
 *
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_self_test ( void )
{
  return __send_basic_command_message (ack, ACK_MESSAGE_SIZE);
}

/**
 *
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_start_timer ( uint16_t timeout_in_minutes )
{
  uint16_t timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  iridium_return_code_t ret = IRIDIUM_SUCCESS;

  if ( tx_timer_change (iridium_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = IRIDIUM_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (iridium_self->timer) != TX_SUCCESS )
  {
    ret = IRIDIUM_TIMER_ERROR;
  }

  return ret;
}

/**
 *
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_stop_timer ( void )
{
  return (tx_timer_deactivate (iridium_self->timer) == TX_SUCCESS) ?
      IRIDIUM_SUCCESS : IRIDIUM_TIMER_ERROR;
}

/**
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_transmit_message ( sbd_message_type_52 *msg )
{
  iridium_return_code_t return_code = IRIDIUM_SUCCESS;
  iridium_return_code_t queue_return_code __attribute__((unused));

  watchdog_check_in (IRIDIUM_THREAD);

  return_code = __internal_transmit_message ((uint8_t*) msg,
                                             sizeof(sbd_message_type_52) - IRIDIUM_CHECKSUM_LENGTH);

  if ( return_code == IRIDIUM_UART_ERROR )
  {
    __cycle_power ();
  }

  return return_code;
}

/**
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t _iridium_transmit_error_message ( char *error_message )
{
  iridium_return_code_t return_code = IRIDIUM_SUCCESS;
//  uint16_t error_msg_str_length = strlen (error_message);
//  uint16_t payload_iterator = 0;
//  float timestamp;
//  float *timestamp_ptr = &timestamp;
//  float *lat_ptr = &iridium_self->current_lat;
//  float *lon_ptr = &iridium_self->current_lon;
//  bool message_tx_success = false;
//
//  watchdog_check_in (IRIDIUM_THREAD);
//  // If the error message is too long, we'll just cut it off
//  if ( error_msg_str_length > ERROR_MESSAGE_MAX_LENGTH - 1 )
//  {
//    error_message[ERROR_MESSAGE_MAX_LENGTH - 1] = 0;
//    error_msg_str_length = ERROR_MESSAGE_MAX_LENGTH - 1;
//  }
//
//  // Assemble the error message payload, start by clearing the whole thing out
//  memset (&(iridium_self->error_message_buffer[0]), 0,
//  IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH);
//  // First byte is message type (99)
//  iridium_self->error_message_buffer[payload_iterator] = ERROR_MESSAGE_TYPE;
//  payload_iterator++;
//  memcpy (&(iridium_self->error_message_buffer[payload_iterator]), error_message, error_msg_str_length);
//  // Set the iterator to the index after the string
//  payload_iterator += ERROR_MESSAGE_MAX_LENGTH;
//  memcpy (&(iridium_self->error_message_buffer[payload_iterator]), lat_ptr, sizeof(iridium_self->current_lat));
//  payload_iterator += sizeof(iridium_self->current_lat);
//  memcpy (&(iridium_self->error_message_buffer[payload_iterator]), lon_ptr, sizeof(iridium_self->current_lon));
//  payload_iterator += sizeof(iridium_self->current_lon);
//  timestamp = iridium_self->get_timestamp ();
//  memcpy (&(iridium_self->error_message_buffer[payload_iterator]), timestamp_ptr, sizeof(float));
//
//  // reset the timer and clear the interrupt flag
//  iridium_self->reset_timer (iridium_self->global_config->iridium_max_transmit_time);
//  // Start the timer in interrupt mode
//  HAL_TIM_Base_Start_IT (iridium_self->timer);
//  // Send the message that was just generated
//  while ( !iridium_self->timer_timeout && !message_tx_success )
//  {
//    return_code = internal_transmit_message ((uint8_t*) &(iridium_self->error_message_buffer[0]),
//    IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE);
//    if ( return_code == IRIDIUM_UART_ERROR )
//    {
//      iridium_self->cycle_power ();
//      iridium_self->reset_uart (IRIDIUM_DEFAULT_BAUD_RATE);
//    }
//    message_tx_success = return_code == IRIDIUM_SUCCESS;
//  }
//
//  // Message failed to send.
//  if ( iridium_self->timer_timeout && !message_tx_success )
//  {
//    // reset the timer and clear the flag for the next time
//    HAL_TIM_Base_Stop_IT (iridium_self->timer);
//    __HAL_TIM_CLEAR_FLAG(iridium_self->timer, TIM_FLAG_UPDATE);
//  }
//
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
 * @return iridium_return_code_t
 */
static iridium_return_code_t __send_basic_command_message ( const char *command,
                                                            uint8_t response_size )
{
  char *needle;
  ULONG actual_flags;

  memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

  if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(command[0]),
                                        strlen (command))
       != UART_OK )
  {
    return IRIDIUM_UART_ERROR;
  }

  if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                       &(iridium_self->response_buffer[0]), response_size)
       != UART_OK )
  {
    return IRIDIUM_UART_ERROR;
  }

  needle = strstr ((char*) &(iridium_self->response_buffer[1]), "OK");
  memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
  return (needle == NULL) ?
      IRIDIUM_COMMAND_RESPONSE_ERROR : IRIDIUM_SUCCESS;
}

/**
 *
 * @return iridium_return_code_t
 */
static iridium_return_code_t __internal_transmit_message ( uint8_t *payload, uint16_t payload_size )
{
  iridium_return_code_t return_code = IRIDIUM_TRANSMIT_TIMEOUT;
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

  // Reset the UART Tx/Rx timeouts
  generic_uart_set_timeout_ticks (&iridium_self->uart_driver, IRIDIUM_MAX_UART_TX_TICKS,
  IRIDIUM_MAX_UART_RX_TICKS_NO_TX);

  while ( !iridium_self->timer_timeout )
  {

    message_response_received = false;
    tx_response_time = 0;

    // get the checksum
    __get_checksum ((uint8_t*) payload, payload_size);

    // Tell the modem we want to send a message
    for ( fail_counter = 0; fail_counter < max_retries && !iridium_self->timer_timeout;
        fail_counter++ )
    {
      watchdog_check_in (IRIDIUM_THREAD);

      if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(load_sbd[0]),
                                            strlen (load_sbd))
           != UART_OK )
      {
        return IRIDIUM_UART_ERROR;
      }

      if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                           &(iridium_self->response_buffer[0]),
                                           SBDWB_READY_RESPONSE_SIZE)
           != UART_OK )
      {
        return IRIDIUM_UART_ERROR;
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
      return_code = IRIDIUM_UART_ERROR;
      continue;
    }

    // Send over the payload + checksum
    for ( fail_counter = 0; fail_counter < max_retries && !iridium_self->timer_timeout;
        fail_counter++ )
    {
      watchdog_check_in (IRIDIUM_THREAD);

      if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(payload[0]),
                                            payload_size + IRIDIUM_CHECKSUM_LENGTH)
           != UART_OK )
      {
        return IRIDIUM_UART_ERROR;
      }

      if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                           &(iridium_self->response_buffer[0]),
                                           SBDWB_LOAD_RESPONSE_SIZE)
           != UART_OK )
      {
        return IRIDIUM_UART_ERROR;
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
      return_code = IRIDIUM_UART_ERROR;
      continue;
    }

    watchdog_check_in (IRIDIUM_THREAD);

    // Need to adjust the timeout for UART reception as the message can take up to 45r seconds to
    // get a response
    generic_uart_set_timeout_ticks (&iridium_self->uart_driver, TX_TIMER_TICKS_PER_SECOND,
    IRIDIUM_MAX_UART_RX_TICKS_TX);

    // Tell the modem to send the message
    if ( iridium_self->uart_driver.write (&iridium_self->uart_driver, (uint8_t*) &(send_sbd[0]),
                                          strlen (send_sbd))
         != UART_OK )
    {
      return IRIDIUM_UART_ERROR;
    }

    if ( iridium_self->uart_driver.read (&iridium_self->uart_driver,
                                         &(iridium_self->response_buffer[0]),
                                         SBDIX_RESPONSE_SIZE)
         != UART_OK )
    {
      return IRIDIUM_UART_ERROR;
    }

    // Reset the UART Tx/Rx timeouts
    generic_uart_set_timeout_ticks (&iridium_self->uart_driver, IRIDIUM_MAX_UART_TX_TICKS,
    IRIDIUM_MAX_UART_RX_TICKS_NO_TX);

    watchdog_check_in (IRIDIUM_THREAD);

    // Grab the MO status
    needle = strstr ((char*) &(iridium_self->response_buffer[0]), sbdix_search_term);
    needle += strlen (sbdix_search_term);
    SBDIX_response_code = atoi (needle);

    if ( SBDIX_response_code <= 4 )
    {
      // Success case
#warning "May need to reimplement timeout as an inpout to __send_basic_command_message."
      __send_basic_command_message (clear_MO, SBDD_RESPONSE_SIZE);
      LOG("Iridium transmission successful.");
      return IRIDIUM_SUCCESS;
    }

    LOG("Iridium transmission unsuccessful. MO status: %d", SBDIX_response_code);

    // If message Tx failed, put the modem to sleep and delay for a total of 30 seconds
    iridium_self->sleep ();
    watchdog_check_in (IRIDIUM_THREAD);
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND * 25);
    watchdog_check_in (IRIDIUM_THREAD);
    iridium_self->wake ();
    iridium_self->charge_caps (TX_TIMER_TICKS_PER_SECOND * 5);
    watchdog_check_in (IRIDIUM_THREAD);

    memset (&(iridium_self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
    return_code = IRIDIUM_TRANSMIT_UNSUCCESSFUL;
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
  tx_thread_sleep (1);
  iridium_self->on ();
  tx_thread_sleep (1);
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

static int32_t __uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Receive_DMA (driver_handle->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, driver_handle->rx_timeout_ticks) != TX_SUCCESS )
  {
    HAL_UART_DMAStop (driver_handle->uart_handle);
    HAL_UART_Abort (driver_handle->uart_handle);
    return UART_ERR;
  }

  return UART_OK;
}

static int32_t __uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Transmit_DMA (driver_handle->uart_handle, write_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, driver_handle->tx_timeout_ticks) != TX_SUCCESS )
  {
    HAL_UART_DMAStop (driver_handle->uart_handle);
    HAL_UART_Abort (driver_handle->uart_handle);
    return UART_ERR;
  }

  return UART_OK;
}


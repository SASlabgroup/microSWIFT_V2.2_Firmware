/*
 * thread_functions.c
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#include "threadx_support.h"
#include "stdbool.h"
#include "tx_api.h"
#include "main.h"
#include "gnss.h"
#include "ct_sensor.h"
#include "iridium.h"
#include "temp_sensor.h"
#include "turbidity_sensor.h"
#include "light_sensor.h"
#include "accelerometer_sensor.h"
#include "iridium.h"

bool startup_procedure ( microSWIFT_configuration *global_config )
{

  // TODO: In subsequent sampling windows, if a non-critical sensor fails, set an error flag, shut
  //       the component and thread down, and continue on.
  // TODO: Add filex thread to the list (core) when the interface is completed

  UINT tx_return;
  ULONG init_success_flags = (RTC_INIT_SUCCESS | GNSS_INIT_SUCCESS | WAVES_THREAD_INIT_SUCCESS
                              | IRIDIUM_INIT_SUCCESS);
  ULONG current_flags;

  // Start core threads
  (void) tx_thread_resume (&gnss_thread);
  (void) tx_thread_resume (&waves_thread);
  (void) tx_thread_resume (&iridium_thread);

  // Start optional component threads if enabled
  if ( global_config->ct_enabled )
  {
    init_success_flags |= CT_INIT_SUCCESS;
    (void) tx_thread_resume (&ct_thread);
  }

  if ( global_config->temperature_enabled )
  {
    init_success_flags |= TEMPERATURE_INIT_SUCCESS;
    (void) tx_thread_resume (&temperature_thread);
  }

  if ( global_config->light_enabled )
  {
    init_success_flags |= LIGHT_INIT_SUCCESS;
    (void) tx_thread_resume (&light_thread);
  }

  if ( global_config->turbidity_enabled )
  {
    init_success_flags |= TURBIDITY_INIT_SUCCESS;
    (void) tx_thread_resume (&turbidity_thread);
  }

  if ( global_config->accelerometer_enabled )
  {
    init_success_flags |= ACCELEROMETER_INIT_SUCCESS;
    (void) tx_thread_resume (&accelerometer_thread);
  }

  // Flash power up sequence (this will also give threads time to execute their init procedures)
  led_sequence (INITIAL_LED_SEQUENCE);

  tx_return = tx_event_flags_get (&initialization_flags, init_success_flags, TX_AND_CLEAR,
                                  &current_flags, STARTUP_SEQUENCE_MAX_WAIT_TICKS);

  return (tx_return == TX_SUCCESS);
}

bool gnss_apply_config ( GNSS *gnss )
{
  int fail_counter = 0, max_retries = 10;
  gnss_error_code_t gnss_return_code;

  gnss->on_off (GPIO_PIN_SET);

  while ( fail_counter < max_retries )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    gnss_return_code = gnss->config ();
    if ( gnss_return_code != GNSS_SUCCESS )
    {
      fail_counter++;
    }
    else
    {
      break;
    }
  }

  if ( gnss_return_code != GNSS_SUCCESS )
  {
    gnss->on_off (GPIO_PIN_RESET);
  }

  return (gnss_return_code == GNSS_SUCCESS);
}

bool ct_and_self_test ( CT *ct )
{
  int32_t fail_counter = 0, max_retries = 10;
  ct_error_code_t ct_return_code;

  ct->on_off (GPIO_PIN_SET);

  while ( fail_counter < max_retries )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    ct_return_code = ct->self_test (false, NULL);
    if ( ct_return_code != CT_SUCCESS )
    {
      fail_counter++;
    }
    else
    {
      break;
    }
  }

  ct->on_off (GPIO_PIN_RESET);

  return (ct_return_code == CT_SUCCESS);
}

bool temperature_self_test ( Temperature *temperature, float *self_test_temp )
{
  int32_t fail_counter = 0, max_retries = 10;
  temperature_error_code_t temp_return_code;

  temperature->on ();

  while ( fail_counter < max_retries )
  {
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    temp_return_code = temperature->self_test ();
    if ( temp_return_code != TEMPERATURE_SUCCESS )
    {
      temperature->reset_i2c ();
      fail_counter++;
    }
    else
    {
      break;
    }
  }

  temperature->off ();

  return (temp_return_code == TEMPERATURE_SUCCESS);
}

bool turbidity_self_test ( void )
{
  return true;
}

bool light_self_test ( void )
{
  return true;
}

bool accelerometer_self_test ( void )
{
  return true;
}

bool iridium_apply_config ( Iridium *iridium )
{
  int32_t fail_counter = 0, max_retries = 10;
  iridium_error_code_t iridium_return_code;

  iridium->on_off (true);

  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->self_test ();
    if ( iridium_return_code != IRIDIUM_SUCCESS )
    {
      iridium->cycle_power ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;
    }
    else
    {
      break;
    }
  }

  if ( iridium_return_code != IRIDIUM_SUCCESS )
  {
    iridium->sleep (GPIO_PIN_RESET);
    return false;
  }

  // Send the configuration settings to the modem
  fail_counter = 0;
  while ( fail_counter < max_retries )
  {
    iridium_return_code = iridium->config ();
    if ( iridium_return_code != IRIDIUM_SUCCESS )
    {
      iridium->cycle_power ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;
    }
    else
    {
      break;
    }
  }

  return (iridium_return_code == IRIDIUM_SUCCESS);
}

bool is_first_sample_window ( void )
{
  return true;
}

ULONG get_current_flags ( TX_EVENT_FLAGS_GROUP *event_flags )
{
  ULONG current_flags = 0;
  (void) tx_event_flags_get (event_flags, ALL_EVENT_FLAGS, TX_OR, &current_flags, TX_NO_WAIT);
  return current_flags;
}

void clear_event_flags ( TX_EVENT_FLAGS_GROUP *event_flags )
{
  ULONG current_flags = 0;
  (void) tx_event_flags_get (event_flags, ALL_EVENT_FLAGS, TX_OR_CLEAR, &current_flags, TX_NO_WAIT);
}

uint32_t ticks_from_milliseconds ( uint32_t milliseconds )
{
  if ( milliseconds == 0 )
  {
    return 0;
  }

  return ((uint32_t) (ceil (((float) milliseconds) / ((1.0f / TX_TIMER_TICKS_PER_SECOND) * 1000.0f))));
}

/**
 * @brief  Static function to flash a sequence of onboard LEDs to indicate
 * success or failure of self-test.
 *
 * @param  sequence:   INITIAL_LED_SEQUENCE
 *                                     TEST_PASSED_LED_SEQUENCE
 *                                     TEST_NON_CIRTICAL_FAULT_LED_SEQUENCE
 *                                     TEST_CRITICAL_FAULT_LED_SEQUENCE
 *
 * @retval Void
 */
void led_sequence ( led_sequence_t sequence )
{
  switch ( sequence )
  {
    case INITIAL_LED_SEQUENCE:
      for ( int i = 0; i < 10; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 4);
      }
      break;

    case TEST_PASSED_LED_SEQUENCE:
      for ( int i = 0; i < 5; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
        HAL_GPIO_WritePin (EXT_LED_GREEN_GPIO_Port, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
      }
      break;

    case TEST_FAILED_LED_SEQUENCE:
      for ( int i = 0; i < 10; i++ )
      {
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_RESET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 2);
        HAL_GPIO_WritePin (EXT_LED_RED_GPIO_Port, EXT_LED_RED_Pin, GPIO_PIN_SET);
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 2);
      }
      break;

    default:
      break;
  }
}

/**
 * @brief  If an error was detected along the way, send an error (Type 99) message.
 *
 * @param  error_flags - retreived error flags
 *
 * @retval void
 */
void send_error_message ( Iridium *iridium, ULONG error_flags )
{
//  iridium_error_code_t return_code;
//  char error_message[ERROR_MESSAGE_MAX_LENGTH] =
//    { 0 };
//  const char *watchdog_reset = "WATCHDOG RESET. ";
//  const char *software_reset = "SOFTWARE RESET. ";
//  const char *gnss_error = "GNSS ERROR. ";
//  const char *gnss_resolution_error = "GNSS RESOLUTION ERROR. ";
//  const char *sample_window_error = "SAMPLE WINDOW ERROR. ";
//  const char *memory_corruption_error = "MEMORY CORRUPTION ERROR. ";
//  const char *memory_alloc_error = "MEMORY ALLOC ERROR. ";
//  const char *dma_error = "DMA ERROR. ";
//  const char *uart_error = "UART ERROR. ";
//  const char *rtc_error = "RTC ERROR. ";
//  const char *flash_success = "WRITE TO FLASH SUCCESSFUL. ";
//  const char *flash_unknown_error = "UNKNOWN FLASH ERROR. ";
//  const char *flash_storage_full = "FLASH STORAGE FULL. ";
//  const char *flash_erase_error = "FLASH ERASE ERROR. ";
//  const char *flash_program_error = "FLASH PROGRAM ERROR. ";
//  char *string_ptr = &(error_message[0]);
//
//  if ( error_flags & WATCHDOG_RESET )
//  {
//
//    if ( (string_ptr - &(error_message[0])) + strlen (watchdog_reset) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, watchdog_reset, strlen (watchdog_reset));
//      string_ptr += strlen (watchdog_reset);
//    }
//
//  }
//
//  if ( error_flags & SOFTWARE_RESET )
//  {
//
//    if ( (string_ptr - &(error_message[0])) + strlen (software_reset) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, software_reset, strlen (software_reset));
//      string_ptr += strlen (software_reset);
//    }
//
//  }
//
//  if ( error_flags & GNSS_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (gnss_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, gnss_error, strlen (gnss_error));
//      string_ptr += strlen (gnss_error);
//    }
//  }
//
//  if ( error_flags & GNSS_RESOLUTION_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (gnss_resolution_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, gnss_resolution_error, strlen (gnss_resolution_error));
//      string_ptr += strlen (gnss_resolution_error);
//    }
//  }
//
//  if ( error_flags & SAMPLE_WINDOW_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (sample_window_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, sample_window_error, strlen (sample_window_error));
//      string_ptr += strlen (sample_window_error);
//    }
//  }
//
//  if ( error_flags & MEMORY_CORRUPTION_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (memory_corruption_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, memory_corruption_error, strlen (memory_corruption_error));
//      string_ptr += strlen (memory_corruption_error);
//    }
//  }
//
//  if ( error_flags & MEMORY_ALLOC_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (memory_alloc_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, memory_alloc_error, strlen (memory_alloc_error));
//      string_ptr += strlen (memory_alloc_error);
//    }
//  }
//
//  if ( error_flags & DMA_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (dma_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, dma_error, strlen (dma_error));
//      string_ptr += strlen (dma_error);
//    }
//  }
//
//  if ( error_flags & UART_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (uart_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, uart_error, strlen (uart_error));
//      string_ptr += strlen (uart_error);
//    }
//  }
//
//  if ( error_flags & RTC_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (rtc_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, rtc_error, strlen (rtc_error));
//      string_ptr += strlen (rtc_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_SUCCESS )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (flash_success) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_success, strlen (flash_success));
//      string_ptr += strlen (flash_success);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_UNKNOWN_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_unknown_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_unknown_error, strlen (flash_unknown_error));
//      string_ptr += strlen (flash_unknown_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_STORAGE_FULL )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_storage_full) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_storage_full, strlen (flash_storage_full));
//      string_ptr += strlen (flash_storage_full);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_ERASE_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0])) + strlen (flash_erase_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_erase_error, strlen (flash_erase_error));
//      string_ptr += strlen (flash_erase_error);
//    }
//  }
//
//  if ( error_flags & FLASH_OPERATION_PROGRAM_ERROR )
//  {
//    if ( (string_ptr - &(error_message[0]))
//         + strlen (flash_program_error) <= ERROR_MESSAGE_MAX_LENGTH )
//    {
//      memcpy (string_ptr, flash_program_error, strlen (flash_program_error));
//      string_ptr += strlen (flash_program_error);
//    }
//  }
//
//  return_code = iridium->transmit_error_message (error_message);
//
//  if ( (return_code == IRIDIUM_SUCCESS) && (device_handles.reset_reason != 0) )
//  {
//    // Only want to send this message once, so clear reset_reason
//    device_handles.reset_reason = 0;
//  }
}

/**
 * @brief  Power down all peripheral FETs and set RF switch to GNSS input
 *
 * @param  void
 *
 * @retval void
 */
void shut_it_all_down ( void )
{
// Shut down Iridium modem
  HAL_GPIO_WritePin (GPIOD, IRIDIUM_OnOff_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (GPIOD, IRIDIUM_FET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (GPIOF, BUS_5V_FET_Pin, GPIO_PIN_RESET);
// Shut down GNSS
  HAL_GPIO_WritePin (GPIOG, GNSS_FET_Pin, GPIO_PIN_RESET);
// Reset RF switch GPIOs. This will set it to be ported to the modem (safe case)
  HAL_GPIO_WritePin (GPIOD, RF_SWITCH_VCTL_Pin, GPIO_PIN_RESET);
// Turn off power to the RF switch
  HAL_GPIO_WritePin (GPIOD, RF_SWITCH_EN_Pin, GPIO_PIN_RESET);
// Shut down CT sensor
  HAL_GPIO_WritePin (GPIOG, CT_FET_Pin, GPIO_PIN_RESET);
}

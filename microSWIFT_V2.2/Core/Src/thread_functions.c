/*
 * thread_functions.c
 *
 *  Created on: Aug 20, 2024
 *      Author: philbush
 */

#include "thread_functions.h"
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

bool startup_procedure ( void )
{

  /*
   *
   *
   *
   *
   *
   *
   *
   *
   *
   * Initial LED Sequence
   *
   * Launch FileX, wait until it reports success
   * Get the sample window counter, any other bookkeeping
   * Launch waves thread, wait until it reports success
   * Init RF switch, port to GNSS
   * Init battery, take reading, shutdown
   * Launch GNSS thread
   * Launch Iridium
   * Launch remaining threads based on configuration
   *
   *
   * Collect the init success flags as TX_AND(CLEAR?) with a wait of some predetermined tick count
   * return self test status
   */

  UINT tx_return;

  tx_return = tx_event_flags_get (&thread_control_flags, FULL_CYCLE_COMPLETE, TX_OR_CLEAR,
                                  &actual_flags, TX_NO_WAIT);
  // If this is a subsequent window, just setup the GNSS, skip the rest
  if ( tx_return == TX_SUCCESS )
  {

    // Increment the sample window counter
    sample_window_counter++;

    register_watchdog_refresh ();

    HAL_GPIO_WritePin (GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);

    // Reset all threads
    tx_return = tx_thread_reset (&gnss_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&gnss_thread);
      tx_thread_reset (&gnss_thread);
    }
#if CT_ENABLED
    tx_return = tx_thread_reset (&ct_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&ct_thread);
      tx_thread_reset (&ct_thread);
    }
#endif

#if TEMPERATURE_ENABLED
    tx_return = tx_thread_reset (&temperature_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&temperature_thread);
      tx_thread_reset (&temperature_thread);
    }
#endif
    tx_return = tx_thread_reset (&waves_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&waves_thread);
      tx_thread_reset (&waves_thread);
    }
    tx_return = tx_thread_reset (&iridium_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&iridium_thread);
      tx_thread_reset (&iridium_thread);
    }
    tx_return = tx_thread_reset (&end_of_cycle_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&end_of_cycle_thread);
      tx_thread_reset (&end_of_cycle_thread);
    }

    // Power up the RF switch
    rf_switch->power_on ();

    // Check if there was a GNSS error. If so, reconfigure device
    tx_return = tx_event_flags_get (&thread_control_flags, GNSS_CONFIG_REQUIRED, TX_OR_CLEAR,
                                    &actual_flags, TX_NO_WAIT);
    if ( tx_return == TX_SUCCESS )
    {
      fail_counter = 0;
      while ( fail_counter < MAX_SELF_TEST_RETRIES )
      {

        register_watchdog_refresh ();

        if ( gnss->config () != GNSS_SUCCESS )
        {
          // Config didn't work, cycle power and try again
          gnss->cycle_power ();
          fail_counter++;
        }
        else
        {
          break;
        }
      }
    }
    // If we couldn't configure the GNSS, send a reset vector
    if ( fail_counter == MAX_SELF_TEST_RETRIES )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }

    // Kick off the GNSS thread
    if ( tx_thread_resume (&gnss_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }
  }

  // This is first time power up, test everything and flash LED sequence
  else
  {
    register_watchdog_refresh ();
    // Flash some lights to let the user know its on and working
    led_sequence (INITIAL_LED_SEQUENCE);

    rf_switch->power_on ();

    register_watchdog_refresh ();

    self_test_status = initial_power_on_self_test ();

    register_watchdog_refresh ();

    // Kick off the GNSS thread
    if ( tx_thread_resume (&gnss_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }
  }

  return self_test_status;
}

/**
 * @brief  Test communication with each peripheral.
 *
 * @param  void
 *
 * @retval SELF_TEST_PASSED
 *             SELF_TEST_NON_CRITICAL_FAULT --> if CT or IMU failed
 *             SELF_TEST_CRITICAL_FAULT --> if GNSS or Iridium modem
 */
self_test_status_t initial_power_on_self_test ( void )
{
  self_test_status_t return_code;
  gnss_error_code_t gnss_return_code;
  iridium_error_code_t iridium_return_code;

#if CT_ENABLED
  ct_error_code_t ct_return_code;
#endif

#if TEMPERATURE_ENABLED
  temperature_error_code_t temp_return_code;
#endif

  int fail_counter;

// Initialize the sample window counter to 0
  sample_window_counter = 0;

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// GNSS STARTUP SEQUENCE /////////////////////////////////////////////
// turn on the GNSS FET
  gnss->on_off (GPIO_PIN_SET);
  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
// Send the configuration commands to the GNSS unit.
  fail_counter = 0;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();

    gnss_return_code = gnss->config ();
    if ( gnss_return_code != GNSS_SUCCESS )
    {
      // Config didn't go through, try again
      fail_counter++;

    }
    else
    {

      break;
    }
  }

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    return_code = SELF_TEST_CRITICAL_FAULT;
    tx_event_flags_set (&error_flags, GNSS_ERROR, TX_OR);
    return return_code;
  }

// If we made it here, the self test passed and we're ready to process messages
  tx_event_flags_set (&thread_control_flags, GNSS_READY, TX_OR);

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////IRIDIUM STARTUP SEQUENCE ///////////////////////////////////////////////
// Only do this on initial power up, else leave it alone!
  iridium->queue_flush ();

#ifdef DEBUGGING_FAST_CYCLE

        iridium->charge_caps(30);

#else

// Turn on the modem and charge up the caps
  iridium->charge_caps (IRIDIUM_INITIAL_CAP_CHARGE_TIME);

#endif

// Send over an ack message and make sure we get a response
  fail_counter = 0;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();
    // See if we can get an ack message from the modem
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

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    return_code = SELF_TEST_CRITICAL_FAULT;
    tx_event_flags_set (&error_flags, MODEM_ERROR, TX_OR);
    return return_code;
  }

// Send the configuration settings to the modem
  fail_counter = 0;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();

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

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    return_code = SELF_TEST_CRITICAL_FAULT;
    tx_event_flags_set (&error_flags, MODEM_ERROR, TX_OR);
    return return_code;
  }

// We'll keep power to the modem but put it to sleep
  iridium->sleep (GPIO_PIN_RESET);

// We got an ack and were able to config the Iridium modem
  tx_event_flags_set (&thread_control_flags, IRIDIUM_READY, TX_OR);

#if CT_ENABLED
  ///////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////// CT STARTUP SEQUENCE ///////////////////////////////////////////////
  // Make sure we get good data from the CT sensor
  // The first message will have a different frame length from a header, so adjust the fail
  // counter appropriately
  fail_counter = -1;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();

    ct_return_code = ct->self_test (false);
    if ( ct_return_code != CT_SUCCESS )
    {

      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;

    }
    else
    {

      return_code = SELF_TEST_PASSED;
      break;
    }
  }

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    return_code = SELF_TEST_NON_CRITICAL_FAULT;
    tx_event_flags_set (&error_flags, CT_ERROR, TX_OR);
  }

  // We can turn off the CT sensor for now
  ct->on_off (GPIO_PIN_RESET);

  // Regardless of if the self-test passed, we'll still set it as ready and try again
  // in the sample window
  tx_event_flags_set (&thread_control_flags, CT_READY, TX_OR);

#else

  return_code = SELF_TEST_PASSED;

#endif

#if TEMPERATURE_ENABLED
  fail_counter = 0;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    temperature->on ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    register_watchdog_refresh ();
    // See if we can get an ack message from the modem
    temp_return_code = temperature->self_test ();
    if ( temp_return_code != TEMPERATURE_SUCCESS )
    {

      temperature->off ();
      temperature->reset_i2c ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;

    }
    else
    {

      break;
    }
  }

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    return_code = SELF_TEST_NON_CRITICAL_FAULT;
    tx_event_flags_set (&error_flags, TEMPERATURE_ERROR, TX_OR);

  }

  temperature->off ();
#endif

  return return_code;
}

bool gnss_apply_config ( GNSS *gnss )
{
  int fail_counter = 0, max_retries = 10;
  gnss_error_code_t gnss_return_code;

  gnss->on_off (GPIO_PIN_SET);

  while ( fail_counter < MAX_SELF_TEST_RETRIES )
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
    ct_return_code = ct->self_test (false);
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

#ifdef DEBUGGING_FAST_CYCLE
        iridium->charge_caps(30);
#else
  // Turn on the modem and charge up the caps
  iridium->charge_caps (IRIDIUM_INITIAL_CAP_CHARGE_TIME);
#endif // #ifdef DEBUGGING_FAST_CYCLE

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

  iridium->sleep (GPIO_PIN_RESET);

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
static void led_sequence ( led_sequence_t sequence )
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
 * @brief  Break out of the GNSS thread and jump to end_of_cycle_thread
 *
 * @param  thread_to_terminate - thread which called this
 *
 * @retval void
 */
void jump_to_end_of_window ( ULONG error_bits_to_set )
{
  gnss->on_off (GPIO_PIN_RESET);
// If there was a GNSS error or it could not resolve in time, set the flag to reconfig next time
  if ( (error_bits_to_set & GNSS_RESOLUTION_ERROR) || (error_bits_to_set & GNSS_ERROR) )
  {
    // Set the event flag so we know to reconfigure in the next window
    tx_event_flags_set (&thread_control_flags, GNSS_CONFIG_REQUIRED, TX_OR);
  }

  tx_event_flags_set (&error_flags, (error_bits_to_set | GNSS_EXITED_EARLY), TX_OR);

// Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (gnss->gnss_uart_handle);
  HAL_DMA_DeInit (gnss->gnss_rx_dma_handle);
  HAL_DMA_DeInit (gnss->gnss_tx_dma_handle);
  HAL_TIM_Base_Stop_IT (gnss->minutes_timer);

  if ( waves_memory_pool_delete () != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  if ( tx_thread_resume (&iridium_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_thread_terminate (&gnss_thread);
}

/**
 * @brief  Break out of the CT thread and jump to Waves thread
 *
 * @param  void
 *
 * @retval void
 */
static void jump_to_waves ( void )
{
  ct->on_off (GPIO_PIN_RESET);
// Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (ct->ct_uart_handle);
  HAL_DMA_DeInit (ct->ct_dma_handle);

  if ( tx_thread_resume (&waves_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_thread_terminate (&ct_thread);
}

/**
 * @brief  If an error was detected along the way, send an error (Type 99) message.
 *
 * @param  error_flags - retreived error flags
 *
 * @retval void
 */
static void send_error_message ( ULONG error_flags )
{
  iridium_error_code_t return_code;
  char error_message[ERROR_MESSAGE_MAX_LENGTH] =
    { 0 };
  const char *watchdog_reset = "WATCHDOG RESET. ";
  const char *software_reset = "SOFTWARE RESET. ";
  const char *gnss_error = "GNSS ERROR. ";
  const char *gnss_resolution_error = "GNSS RESOLUTION ERROR. ";
  const char *sample_window_error = "SAMPLE WINDOW ERROR. ";
  const char *memory_corruption_error = "MEMORY CORRUPTION ERROR. ";
  const char *memory_alloc_error = "MEMORY ALLOC ERROR. ";
  const char *dma_error = "DMA ERROR. ";
  const char *uart_error = "UART ERROR. ";
  const char *rtc_error = "RTC ERROR. ";
  const char *flash_success = "WRITE TO FLASH SUCCESSFUL. ";
  const char *flash_unknown_error = "UNKNOWN FLASH ERROR. ";
  const char *flash_storage_full = "FLASH STORAGE FULL. ";
  const char *flash_erase_error = "FLASH ERASE ERROR. ";
  const char *flash_program_error = "FLASH PROGRAM ERROR. ";
  char *string_ptr = &(error_message[0]);

  if ( error_flags & WATCHDOG_RESET )
  {

    if ( (string_ptr - &(error_message[0])) + strlen (watchdog_reset) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, watchdog_reset, strlen (watchdog_reset));
      string_ptr += strlen (watchdog_reset);
    }

  }

  if ( error_flags & SOFTWARE_RESET )
  {

    if ( (string_ptr - &(error_message[0])) + strlen (software_reset) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, software_reset, strlen (software_reset));
      string_ptr += strlen (software_reset);
    }

  }

  if ( error_flags & GNSS_ERROR )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (gnss_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, gnss_error, strlen (gnss_error));
      string_ptr += strlen (gnss_error);
    }
  }

  if ( error_flags & GNSS_RESOLUTION_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (gnss_resolution_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, gnss_resolution_error, strlen (gnss_resolution_error));
      string_ptr += strlen (gnss_resolution_error);
    }
  }

  if ( error_flags & SAMPLE_WINDOW_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (sample_window_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, sample_window_error, strlen (sample_window_error));
      string_ptr += strlen (sample_window_error);
    }
  }

  if ( error_flags & MEMORY_CORRUPTION_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (memory_corruption_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, memory_corruption_error, strlen (memory_corruption_error));
      string_ptr += strlen (memory_corruption_error);
    }
  }

  if ( error_flags & MEMORY_ALLOC_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (memory_alloc_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, memory_alloc_error, strlen (memory_alloc_error));
      string_ptr += strlen (memory_alloc_error);
    }
  }

  if ( error_flags & DMA_ERROR )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (dma_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, dma_error, strlen (dma_error));
      string_ptr += strlen (dma_error);
    }
  }

  if ( error_flags & UART_ERROR )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (uart_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, uart_error, strlen (uart_error));
      string_ptr += strlen (uart_error);
    }
  }

  if ( error_flags & RTC_ERROR )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (rtc_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, rtc_error, strlen (rtc_error));
      string_ptr += strlen (rtc_error);
    }
  }

  if ( error_flags & FLASH_OPERATION_SUCCESS )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (flash_success) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, flash_success, strlen (flash_success));
      string_ptr += strlen (flash_success);
    }
  }

  if ( error_flags & FLASH_OPERATION_UNKNOWN_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (flash_unknown_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, flash_unknown_error, strlen (flash_unknown_error));
      string_ptr += strlen (flash_unknown_error);
    }
  }

  if ( error_flags & FLASH_OPERATION_STORAGE_FULL )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (flash_storage_full) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, flash_storage_full, strlen (flash_storage_full));
      string_ptr += strlen (flash_storage_full);
    }
  }

  if ( error_flags & FLASH_OPERATION_ERASE_ERROR )
  {
    if ( (string_ptr - &(error_message[0])) + strlen (flash_erase_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, flash_erase_error, strlen (flash_erase_error));
      string_ptr += strlen (flash_erase_error);
    }
  }

  if ( error_flags & FLASH_OPERATION_PROGRAM_ERROR )
  {
    if ( (string_ptr - &(error_message[0]))
         + strlen (flash_program_error) <= ERROR_MESSAGE_MAX_LENGTH )
    {
      memcpy (string_ptr, flash_program_error, strlen (flash_program_error));
      string_ptr += strlen (flash_program_error);
    }
  }

  return_code = iridium->transmit_error_message (error_message);

  if ( (return_code == IRIDIUM_SUCCESS) && (device_handles.reset_reason != 0) )
  {
    // Only want to send this message once, so clear reset_reason
    device_handles.reset_reason = 0;
  }
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

/*
 * controller.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include "controller.h"
#include "threadx_support.h"
#include "logger.h"
#include "persistent_ram.h"
#include "ext_rtc_api.h"
#include "NEDWaves/rtwhalf.h"
#include "ct_sensor.h"

// @formatter:off
static Control  *controller_self;

// Struct functions
static bool     _control_startup_procedure( void );
static void     _control_shutdown_procedure( void );
static real16_T _control_get_battery_voltage( void );
static void     _control_shutdown_all_peripherals ( void );
static void     _control_shutdown_all_interfaces ( void );
static void     _control_enter_processor_standby_mode ( void );
static void     _control_manage_state ( void );
static void     _control_monitor_and_handle_errors ( void );

// Helper functions
static void     __get_alarm_settings_from_time(struct tm* time, rtc_alarm_struct *alarm);
static void     __handle_rtc_error( void );
static void     __handle_gnss_error( ULONG error_flag );
static void     __handle_ct_error( ULONG error_flag );
static void     __handle_temperature_error( ULONG error_flag );
static void     __handle_turbidity_error( ULONG error_flag );
static void     __handle_light_error( ULONG error_flag );
static void     __handle_accelerometer_error( ULONG error_flag );
static void     __handle_waves_error( void );
static void     __handle_iridium_error( ULONG error_flags );
static void     __handle_file_system_error( void );
static void     __handle_misc_error ( ULONG error_flag  );

// Static helper functions

// @formatter:on

void controller_init ( Control *struct_ptr, microSWIFT_configuration *global_config,
                       Thread_Handles *thread_handles, TX_EVENT_FLAGS_GROUP *error_flags,
                       TX_EVENT_FLAGS_GROUP *init_flags, TX_EVENT_FLAGS_GROUP *irq_flags,
                       TX_EVENT_FLAGS_GROUP *complete_flags, TX_TIMER *timer,
                       ADC_HandleTypeDef *battery_adc_handle, sbd_message_type_52 *current_message )
{
  // Init self
  controller_self = struct_ptr;

  controller_self->global_config = global_config;
  controller_self->thread_handles = thread_handles;
  controller_self->error_flags = error_flags;
  controller_self->init_flags = init_flags;
  controller_self->irq_flags = irq_flags;
  controller_self->complete_flags = complete_flags;
  controller_self->timer = timer;
  controller_self->current_message = current_message;
  controller_self->error_detected = false;

  controller_self->thread_status.gnss_complete = false;
  controller_self->thread_status.waves_complete = false;
  controller_self->thread_status.iridium_complete = false;
  controller_self->thread_status.ct_complete = !controller_self->global_config->ct_enabled;
  controller_self->thread_status.temperature_complete = !controller_self->global_config
      ->temperature_enabled;
  controller_self->thread_status.light_complete = !controller_self->global_config->light_enabled;
  controller_self->thread_status.turbidity_complete = !controller_self->global_config
      ->turbidity_enabled;
  controller_self->thread_status.accelerometer_complete = !controller_self->global_config
      ->accelerometer_enabled;

  controller_self->startup_procedure = _control_startup_procedure;
  controller_self->shutdown_procedure = _control_shutdown_procedure;
  controller_self->get_battery_voltage = _control_get_battery_voltage;
  controller_self->shutdown_all_peripherals = _control_shutdown_all_peripherals;
  controller_self->shutdown_all_interfaces = _control_shutdown_all_interfaces;
  controller_self->enter_processor_standby_mode = _control_enter_processor_standby_mode;
  controller_self->manage_state = _control_manage_state;
  controller_self->monitor_and_handle_errors = _control_monitor_and_handle_errors;

  // Init battery
  battery_init (&controller_self->battery, battery_adc_handle, controller_self->irq_flags);
  // Init RF switch
  rf_switch_init (&controller_self->rf_switch);
}

void control_timer_expired ( ULONG expiration_input )
{
  (void) expiration_input;
}

static bool _control_startup_procedure ( void )
{
  UINT tx_return;
  ULONG init_success_flags = (RTC_INIT_SUCCESS | GNSS_INIT_SUCCESS | WAVES_THREAD_INIT_SUCCESS
                              | IRIDIUM_INIT_SUCCESS);
  ULONG current_flags;
  uint32_t reset_reason;
  real16_T voltage =
    { 0 };
  bool initial_powerup = is_first_sample_window ();

#warning "In subsequent sampling windows, if a non-critical sensor fails, set an error flag, shut\
         the component and thread down, and continue on."
#warning "Add filex thread to the list (core) when the interface is completed"

  if ( !initial_powerup )
  {
    // Set the watchdog reset or software reset flags
    reset_reason = HAL_RCC_GetResetSource ();

    if ( reset_reason & RCC_RESET_FLAG_PIN )
    {
      LOG("Watchdog reset occured.");
      tx_event_flags_set (&error_flags, WATCHDOG_RESET, TX_OR);
    }

    if ( reset_reason & RCC_RESET_FLAG_SW )
    {
      LOG("Software reset occured.");
      tx_event_flags_set (&error_flags, SOFTWARE_RESET, TX_OR);
    }
  }

  tx_thread_sleep (2);

  // Set the RF switch to GNSS port
  controller_self->rf_switch.power_on ();
  controller_self->rf_switch.set_gnss_port ();

  // Start core threads
  (void) tx_thread_resume (controller_self->thread_handles->waves_thread);
  (void) tx_thread_resume (controller_self->thread_handles->gnss_thread);
  (void) tx_thread_resume (controller_self->thread_handles->iridium_thread);

// Start optional component threads if enabled
  if ( controller_self->global_config->ct_enabled )
  {
    init_success_flags |= CT_INIT_SUCCESS;
    (void) tx_thread_resume (controller_self->thread_handles->ct_thread);
  }

  if ( controller_self->global_config->temperature_enabled )
  {
    init_success_flags |= TEMPERATURE_INIT_SUCCESS;
    (void) tx_thread_resume (controller_self->thread_handles->temperature_thread);
  }

  if ( controller_self->global_config->light_enabled )
  {
    init_success_flags |= LIGHT_INIT_SUCCESS;
    (void) tx_thread_resume (controller_self->thread_handles->light_thread);
  }

  if ( controller_self->global_config->turbidity_enabled )
  {
    init_success_flags |= TURBIDITY_INIT_SUCCESS;
    (void) tx_thread_resume (controller_self->thread_handles->turbidity_thread);
  }

  if ( controller_self->global_config->accelerometer_enabled )
  {
    init_success_flags |= ACCELEROMETER_INIT_SUCCESS;
    (void) tx_thread_resume (controller_self->thread_handles->accelerometer_thread);
  }

  // Flash power up sequence (this will also give threads time to execute their init procedures)
  if ( initial_powerup )
  {
    led_sequence (INITIAL_LED_SEQUENCE);
  }

  // Grab the battery voltage
  voltage = controller_self->get_battery_voltage ();
  memcpy (&controller_self->current_message->mean_voltage, &voltage, sizeof(real16_T));

  battery_deinit ();

  tx_return = tx_event_flags_get (controller_self->init_flags, init_success_flags, TX_AND_CLEAR,
                                  &current_flags, STARTUP_SEQUENCE_MAX_WAIT_TICKS);
  return (tx_return == TX_SUCCESS);
}

static void _control_shutdown_procedure ( void )
{
  rtc_alarm_struct alarm_settings;
  struct tm time_now;

  // Make sure everything is shut down
  controller_self->shutdown_all_peripherals ();

  // Make extra sure the alarm flag is cleared
  if ( rtc_server_clear_flag (ALARM_FLAG, CONTROL_REQUEST_COMPLETE) != RTC_SUCCESS )
  {
    HAL_NVIC_SystemReset ();
  }

  tx_thread_sleep (1);

  // And the alarm interrupt line on the RTC is high (off)
  if ( HAL_GPIO_ReadPin (RTC_INT_A_GPIO_Port, RTC_INT_A_Pin) != GPIO_PIN_SET )
  {
    HAL_NVIC_SystemReset ();
  }

  // Get the time so we can set the alarm
  if ( rtc_server_get_time (&time_now, CONTROL_REQUEST_COMPLETE) != RTC_SUCCESS )
  {
    HAL_NVIC_SystemReset ();
  }

  // Set the alarm
  __get_alarm_settings_from_time (&time_now, &alarm_settings);
  if ( rtc_server_set_alarm (alarm_settings, CONTROL_REQUEST_COMPLETE) != RTC_SUCCESS )
  {
    HAL_NVIC_SystemReset ();
  }

  LOG("All threads complete. Entering processor standby mode. Alarm set for %02d:%02d:%02d UTC.",
      (int ) alarm_settings.alarm_hour, (int ) alarm_settings.alarm_minute,
      (int ) alarm_settings.alarm_second);

  persistent_ram_increment_sample_window_counter ();

  tx_thread_sleep (2);

  // Deinit all enabled peripherals
  controller_self->shutdown_all_interfaces ();

  // Enter standby mode -- processor will be woken by RTC alarm
  controller_self->enter_processor_standby_mode ();
}

static real16_T _control_get_battery_voltage ( void )
{
  real16_T battery_voltage =
    { BATTERY_ERROR_VOLTAGE_VALUE };
  battery_return_code_t ret = BATTERY_SUCCESS;

  ret = controller_self->battery.get_voltage (&battery_voltage);
  if ( ret != BATTERY_SUCCESS )
  {
    switch ( ret )
    {
      case BATTERY_ADC_ERROR:
        LOG("Battery ADC error, unable to obtain battery voltage");
        break;

      case BATTERY_TIMEOUT_ERROR:
        LOG("Battery ADC timed out, unable to obtain battery voltage");
        break;

      default:
        LOG("Unknown battery error, unable to obtain battery voltage");
        break;
    }
  }
  else
  {
    LOG("Battery Voltage = %2.2f", halfToFloat (battery_voltage));
  }

  return battery_voltage;
}

static void _control_shutdown_all_peripherals ( void )
{
  // Shut down Iridium modem
  HAL_GPIO_WritePin (IRIDIUM_OnOff_GPIO_Port, IRIDIUM_OnOff_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (IRIDIUM_FET_GPIO_Port, IRIDIUM_FET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (BUS_5V_FET_GPIO_Port, BUS_5V_FET_Pin, GPIO_PIN_RESET);
  // Shut down GNSS
  HAL_GPIO_WritePin (GNSS_FET_GPIO_Port, GNSS_FET_Pin, GPIO_PIN_RESET);
  // Reset RF switch GPIOs. This will set it to be ported to the modem (safe case)
  HAL_GPIO_WritePin (RF_SWITCH_VCTL_GPIO_Port, RF_SWITCH_VCTL_Pin, GPIO_PIN_RESET);
  // Turn off power to the RF switch
  HAL_GPIO_WritePin (RF_SWITCH_EN_GPIO_Port, RF_SWITCH_EN_Pin, GPIO_PIN_RESET);
  // Shut down CT sensor
  HAL_GPIO_WritePin (CT_FET_GPIO_Port, CT_FET_Pin, GPIO_PIN_RESET);

#warning "Make sure all devices are covered here."
}

static void _control_shutdown_all_interfaces ( void )
{
  uart4_deinit ();
  uart5_deinit ();
  usart1_deinit ();
  usart2_deinit ();
  usart3_deinit ();
  usart6_deinit ();

  spi1_deinit ();
  spi2_deinit ();
  spi3_deinit ();

  battery_deinit ();
}

static void _control_enter_processor_standby_mode ( void )
{
  __IO uint32_t dummy;

  // Disable all non-relevant interrupts
  for ( int i = 0; i < HSPI1_IRQn; i++ )
  {
    HAL_NVIC_DisableIRQ (i);
    HAL_NVIC_ClearPendingIRQ (i);
  }

  // Make sure the IRQ is enabled for the wakeup pin
  HAL_NVIC_SetPriority (PVD_PVM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (PVD_PVM_IRQn);

  // Retain all SRAM2 contents
  HAL_PWREx_EnableSRAM2ContentStandbyRetention (PWR_SRAM2_FULL_STANDBY);

  // Enable power clock
  __HAL_RCC_PWR_CLK_ENABLE();

  // Ensure RTC is disabled
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_RTCEN);

  // Clear the backup ram retention bit
  CLEAR_BIT(PWR->BDCR1, PWR_BDCR1_BREN);

  // Make sure the RTC INT_B pin is being pulled up (open drain on RTC)
  HAL_PWREx_EnablePullUpPullDownConfig ();
  HAL_PWREx_EnableGPIOPullUp (PWR_GPIO_B, (1 << 2));

  // PWR_WAKEUP_PIN1_LOW_1 = PB2 --> RTC INT_B Low Polarity
  HAL_PWR_EnableWakeUpPin (PWR_WAKEUP_PIN1_LOW_1);

  // Clear the stop mode and standby mode flags
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
  __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG1);

#ifndef DEBUG
  DBGMCU->CR = 0; // Disable debug, trace and IWDG in low-power modes
#else
  HAL_EnableDBGStandbyMode ();
#endif

  /* Select Standby mode */
  MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_2);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  // Make sure register operations are complete
  dummy = PWR->SR;
  dummy = PWR->CR1;
  dummy = SCB->SCR;

  // Enter low-power mode
  while ( 1 )
  {
    __DSB ();
    __WFI();
  }
}

static void _control_manage_state ( void )
{
#warning "Add a timeout determined by duty cycle. That way, if a thread doesn't finish in time, it can be force quit. "

  ULONG current_flags;
  UINT ret = TX_SUCCESS;

  // @formatter:on
  bool iridium_ready = false;

  (void) tx_event_flags_get (controller_self->complete_flags, ALL_EVENT_FLAGS, TX_OR_CLEAR,
                             &current_flags, TX_NO_WAIT);

  // Exit early case
  if ( current_flags == 0 )
  {
    return;
  }

  // When the GNSS sample window is close to being done, kick off CT or Temperature sensor sampling
  // Either of these (or both) will complete in less than 2 mins
  if ( current_flags & GNSS_TWO_MINS_OUT_FROM_COMPLETION )
  {
    if ( !controller_self->thread_status.ct_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->ct_thread);
    }

    if ( !controller_self->thread_status.temperature_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->temperature_thread);
    }
  }

  // When the GNSS thread is complete, we can run the Waves algo
  if ( current_flags & GNSS_THREAD_COMPLETED_SUCCESSFULLY )
  {
    controller_self->thread_status.gnss_complete = true;
    ret |= tx_thread_resume (controller_self->thread_handles->waves_thread);
  }

  // If GNSS thread errors out, do not run waves
  if ( current_flags & GNSS_THREAD_COMPLETED_WITH_ERRORS )
  {
    controller_self->thread_status.gnss_complete = true;

    if ( !controller_self->thread_status.ct_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->ct_thread);
    }

    if ( !controller_self->thread_status.temperature_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->temperature_thread);
    }
  }

  if ( (current_flags & CT_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & CT_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.ct_complete = true;
  }

  if ( (current_flags & TEMPERATURE_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & TEMPERATURE_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.temperature_complete = true;
  }

  if ( (current_flags & TURBIDITY_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & TURBIDITY_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.temperature_complete = true;
  }

  if ( (current_flags & LIGHT_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & LIGHT_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.light_complete = true;
  }

  if ( (current_flags & ACCELEROMETER_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & ACCELEROMETER_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.accelerometer_complete = true;
  }

  if ( (current_flags & WAVES_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & WAVES_THREAD_COMPLETED_WITH_ERRORS) )
  {
    controller_self->thread_status.waves_complete = true;
  }

  iridium_ready = controller_self->thread_status.gnss_complete
                  && controller_self->thread_status.ct_complete
                  && controller_self->thread_status.temperature_complete
                  && controller_self->thread_status.light_complete
                  && controller_self->thread_status.turbidity_complete
                  && controller_self->thread_status.accelerometer_complete
                  && controller_self->thread_status.waves_complete;

  if ( iridium_ready )
  {
    controller_self->rf_switch.set_iridium_port ();
    ret |= tx_thread_resume (controller_self->thread_handles->iridium_thread);
  }

  controller_self->thread_status.iridium_complete = (current_flags
                                                     & IRIDIUM_THREAD_COMPLETED_SUCCESSFULLY)
                                                    | (current_flags
                                                       & IRIDIUM_THREAD_COMPLETED_WITH_ERRORS);

  if ( controller_self->thread_status.iridium_complete )
  {
    controller_self->shutdown_procedure ();
  }

  if ( ret != TX_SUCCESS )
  {
    controller_self->shutdown_all_peripherals ();
    persistent_ram_deinit ();
    HAL_Delay (10);
    HAL_NVIC_SystemReset ();
  }
}

static void _control_monitor_and_handle_errors ( void )
{
  ULONG current_flags;
  ULONG gnss_errors, ct_errors, temperature_errors, light_errors, turbidity_errors,
      accelerometer_errors, waves_errors, iridium_errors, file_system_errors, rtc_errors,
      misc_errors;

  // Get the error flags
  (void) tx_event_flags_get (controller_self->error_flags, ALL_EVENT_FLAGS, TX_OR_CLEAR,
                             &current_flags, TX_NO_WAIT);

  // Exit early case
  if ( current_flags == 0 )
  {
    return;
  }

  gnss_errors = current_flags
                & (GNSS_INIT_FAILED | GNSS_CONFIGURATION_FAILED | GNSS_RESOLUTION_ERROR
                   | GNSS_TOO_MANY_PARTIAL_MSGS | GNSS_SAMPLE_WINDOW_TIMEOUT
                   | GNSS_FRAME_SYNC_FAILED | GNSS_SAMPLE_WINDOW_ERROR);
  ct_errors = current_flags & (CT_INIT_FAILED | CT_SELF_TEST_FAILED | CT_SAMPLING_ERROR);
  temperature_errors = current_flags
                       & (TEMPERATURE_INIT_FAILED | TEMPERATURE_SELF_TEST_FAILED
                          | TEMPERATURE_SAMPLING_ERROR);
  light_errors = current_flags
                 & (LIGHT_INIT_FAILED | LIGHT_SELF_TEST_FAILED | LIGHT_SAMPLING_ERROR);
  turbidity_errors = current_flags
                     & (TURBIDITY_INIT_FAILED | TURBIDITY_SELF_TEST_FAILED
                        | TURBIDITY_SAMPLING_ERROR);
  accelerometer_errors = current_flags
                         & (ACCELEROMETER_INIT_FAILED | ACCELEROMETER_SELF_TEST_FAILED
                            | ACCELEROMETER_SAMPLING_ERROR);
  iridium_errors = current_flags & (IRIDIUM_INIT_ERROR | IRIDIUM_UART_COMMS_ERROR);
  waves_errors = current_flags & (WAVES_INIT_FAILED);
  file_system_errors = current_flags & (FILE_SYSTEM_ERROR);
  rtc_errors = current_flags & (RTC_ERROR);
  misc_errors = current_flags & (WATCHDOG_RESET | SOFTWARE_RESET | MEMORY_CORRUPTION_ERROR);

  if ( gnss_errors )
  {
    __handle_gnss_error (gnss_errors);
  }

  if ( iridium_errors )
  {
    __handle_iridium_error (iridium_errors);
  }

  if ( rtc_errors )
  {
    __handle_rtc_error ();
  }

  if ( ct_errors )
  {
    __handle_ct_error (ct_errors);
  }

  if ( temperature_errors )
  {
    __handle_temperature_error (temperature_errors);
  }

  if ( light_errors )
  {
    __handle_light_error (light_errors);
  }

  if ( turbidity_errors )
  {
    __handle_turbidity_error (turbidity_errors);
  }

  if ( accelerometer_errors )
  {
    __handle_accelerometer_error (accelerometer_errors);
  }

  if ( waves_errors )
  {
    __handle_waves_error ();
  }

  if ( file_system_errors )
  {
    __handle_file_system_error ();
  }

  if ( misc_errors )
  {
    __handle_misc_error (misc_errors);
  }

}

static void __get_alarm_settings_from_time ( struct tm *time, rtc_alarm_struct *alarm )
{
  alarm->day_alarm_en = false;
  alarm->hour_alarm_en = true;
  alarm->minute_alarm_en = true;
  alarm->second_alarm_en = true;
  alarm->weekday_alarm_en = false;
  alarm->alarm_second = 0;

  // Testing
  alarm->alarm_second = (time->tm_sec + 20) % 60;
  alarm->alarm_minute = (alarm->alarm_second < time->tm_sec) ?
      (time->tm_min + 1) % 60 : time->tm_min;
  alarm->alarm_hour = (alarm->alarm_minute == 0) ?
      (time->tm_hour + 1) % 24 : time->tm_hour;
  return;

  if ( controller_self->global_config->windows_per_hour == 0 )
  {
    alarm->alarm_minute = 0;
    alarm->alarm_hour = (time->tm_hour + 1) % 24;
  }
  else
  {
    alarm->alarm_minute = (time->tm_min > 30) ?
        0 : 30;
    alarm->alarm_hour = (alarm->alarm_minute == 0) ?
        (time->tm_hour + 1) % 24 : time->tm_hour;
  }
}

static void __handle_rtc_error ( void )
{
  char *error_str = "RTC error detected.";
  persistent_ram_log_error_string (error_str);

  controller_self->shutdown_all_peripherals ();
  controller_self->shutdown_all_interfaces ();

  tx_thread_sleep (10);
  HAL_NVIC_SystemReset ();
}

static void __handle_gnss_error ( ULONG error_flags )
{
  // Set all the fields of the SBD message to error values
  memset (controller_self->current_message, 0, sizeof(sbd_message_type_52));

  // Set the GNSS_THREAD_COMPLETED_WITH_ERRORS and WAVES_THREAD_COMPLETED_WITH_ERRORS flags to prevent waves thread from running
  (void) tx_event_flags_set (
      controller_self->complete_flags,
      (GNSS_THREAD_COMPLETED_WITH_ERRORS | WAVES_THREAD_COMPLETED_WITH_ERRORS), TX_OR);

  // Terminate the GNSS thread
  (void) tx_thread_suspend (controller_self->thread_handles->gnss_thread);
  (void) tx_thread_terminate (controller_self->thread_handles->gnss_thread);
}

static void __handle_ct_error ( ULONG error_flag )
{
  // Shut down CT sensor
  HAL_GPIO_WritePin (CT_FET_GPIO_Port, CT_FET_Pin, GPIO_PIN_RESET);

  memset (&controller_self->current_message->mean_temp, CT_VALUES_ERROR_CODE, sizeof(real16_T));
  memset (&controller_self->current_message->mean_salinity, CT_VALUES_ERROR_CODE, sizeof(real16_T));

  persistent_ram_log_error_string ("CT error.");

  // Set the CT_COMPLETED_WITH_ERRORS flag
  (void) tx_event_flags_set (controller_self->complete_flags, CT_THREAD_COMPLETED_WITH_ERRORS,
  TX_OR);

  (void) tx_thread_suspend (controller_self->thread_handles->ct_thread);
  (void) tx_thread_terminate (controller_self->thread_handles->ct_thread);

}
static void __handle_temperature_error ( ULONG error_flag )
{
  /*
   * TODO:
   *    [ ] Turn off the temperature sensor
   *    [ ] Set the temperature field to error code, salinity field to 0
   *    [ ] Log the error in the error message buffer
   *    [ ] Set the TEMPERATURE_THREAD_COMPLETED_WITH_ERORRS flag
   *    [ ] Terminate the Temperature thread
   *    [ ] Continue application logic
   */
#warning" Shut down Temperature sensor here "
//  HAL_GPIO_WritePin (TEMP_FET_GPIO_Port, TEMP_FET_Pin, GPIO_PIN_RESET);

  memset (&controller_self->current_message->mean_temp, CT_VALUES_ERROR_CODE, sizeof(real16_T));

  persistent_ram_log_error_string ("Temperature error.");

  // Set the TEMPERATURE_COMPLETED_WITH_ERRORS flag
  (void) tx_event_flags_set (controller_self->complete_flags,
                             TEMPERATURE_THREAD_COMPLETED_WITH_ERRORS, TX_OR);

  (void) tx_thread_suspend (controller_self->thread_handles->temperature_thread);
  (void) tx_thread_terminate (controller_self->thread_handles->temperature_thread);

}
static void __handle_turbidity_error ( ULONG error_flag )
{
  /*
   * TODO:
   *    [ ] Turn off the turbidity sensor
   *    [ ] Set fields in the SBD message to error values
   *    [ ] Log the error in the error message buffer
   *    [ ] Set the TURBIDITY_THREAD_COMPLETED_WITH_ERORRS flag
   *    [ ] Terminate the turbidity thread
   *    [ ] Continue application logic
   */
}
static void __handle_light_error ( ULONG error_flag )
{
  /*
   * TODO:
   *    [ ] Turn off the light sensor
   *    [ ] Set fields in the SBD message to error values
   *    [ ] Log the error in the error message buffer
   *    [ ] Set the LIGHT_THREAD_COMPLETED_WITH_ERORRS flag
   *    [ ] Terminate the light thread
   *    [ ] Continue application logic
   */
}
static void __handle_accelerometer_error ( ULONG error_flag )
{
  /*
   * TODO:
   *    [ ] Turn off the accelerometer sensor
   *    [ ] Set fields in the SBD message to error values
   *    [ ] Log the error in the error message buffer
   *    [ ] Set the ACCELEROMETER_THREAD_COMPLETED_WITH_ERORRS flag
   *    [ ] Terminate the accelerometer thread
   *    [ ] Continue application logic
   */
}
static void __handle_waves_error ( void )
{
  // Set all the fields of the SBD message to error values
  memset (controller_self->current_message, 0, sizeof(sbd_message_type_52));

  persistent_ram_log_error_string ("Waves memory pool failed to initialize.");

  (void) tx_event_flags_set (controller_self->complete_flags, WAVES_THREAD_COMPLETED_WITH_ERRORS,
  TX_OR);

  (void) tx_thread_suspend (controller_self->thread_handles->waves_thread);
  (void) tx_thread_terminate (controller_self->thread_handles->waves_thread);

}
static void __handle_iridium_error ( ULONG error_flag )
{
  // Shut down the modem
  HAL_GPIO_WritePin (IRIDIUM_OnOff_GPIO_Port, IRIDIUM_OnOff_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (IRIDIUM_FET_GPIO_Port, IRIDIUM_FET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin (BUS_5V_FET_GPIO_Port, BUS_5V_FET_Pin, GPIO_PIN_RESET);

  persistent_ram_log_error_string ("MOdem error detected.");

  (void) tx_event_flags_set (controller_self->complete_flags, IRIDIUM_THREAD_COMPLETED_WITH_ERRORS,
  TX_OR);

  (void) tx_thread_suspend (controller_self->thread_handles->iridium_thread);
  (void) tx_thread_terminate (controller_self->thread_handles->iridium_thread);
}
static void __handle_file_system_error ( void )
{
  /*
   * TODO:
   *    [ ] Unknown
   */
}
static void __handle_misc_error ( ULONG error_flag )
{
  char *error_str;

  if ( error_flag & WATCHDOG_RESET )
  {
    error_str = "Watchdog reset occured.";
    persistent_ram_log_error_string (error_str);
    return;
  }

  if ( error_flag & SOFTWARE_RESET )
  {
    error_str = "Software reset occured.";
    persistent_ram_log_error_string (error_str);
    return;
  }

  if ( error_flag & MEMORY_CORRUPTION_ERROR )
  {
    error_str = "Memory corruption detected.";
    LOG(error_str);

    controller_self->shutdown_all_peripherals ();
    controller_self->shutdown_all_interfaces ();
    // Clear out persistent ram if memory cannot be trusted
    persistent_ram_deinit ();
    tx_thread_sleep (10);
    HAL_NVIC_SystemReset ();
  }
}

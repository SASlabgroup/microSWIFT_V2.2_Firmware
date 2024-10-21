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

// @formatter:off
static Control  *controller_self;

// Struct functions
static bool     _control_startup_procedure( void );
static void     _control_shutdown_procedure( void );
static real16_T _control_get_battery_voltage( void );
static void     _control_shut_down_all_peripherals ( void );
static void     _control_enter_processor_standby_mode ( void );
static void     _control_manage_state ( void );
static void     _control_monitor_and_handle_errors ( void );

// Helper functions
static void     __get_alarm_settings_from_time(struct tm* time, rtc_alarm_struct *alarm);
static void     __handle_rtc_error( void );
static void     __handle_gnss_error( ULONG error_flags );
static void     __handle_ct_error( void );
static void     __handle_temperature_error( void );
static void     __handle_turbidity_error( void );
static void     __handle_light_error( void );
static void     __handle_accelerometer_error( void );
static void     __handle_waves_error( void );
static void     __handle_iridium_error( ULONG error_flags );
static void     __handle_file_system_error( void );

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

  controller_self->startup_procedure = _control_startup_procedure;
  controller_self->shutdown_procedure = _control_shutdown_procedure;
  controller_self->get_battery_voltage = _control_get_battery_voltage;
  controller_self->shutdown_all_pheripherals = _control_shut_down_all_peripherals;
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

#warning "In subsequent sampling windows, if a non-critical sensor fails, set an error flag, shut\
         the component and thread down, and continue on."
#warning "Add filex thread to the list (core) when the interface is completed"

  // Set the watchdog reset or software reset flags
  reset_reason = HAL_RCC_GetResetSource ();

  if ( reset_reason & RCC_RESET_FLAG_PIN )
  {
    tx_event_flags_set (&error_flags, WATCHDOG_RESET, TX_OR);
  }

  if ( reset_reason & RCC_RESET_FLAG_SW )
  {
    tx_event_flags_set (&error_flags, SOFTWARE_RESET, TX_OR);
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
  if ( is_first_sample_window () )
  {
    led_sequence (INITIAL_LED_SEQUENCE);
  }

  tx_return = tx_event_flags_get (controller_self->init_flags, init_success_flags, TX_AND_CLEAR,
                                  &current_flags, STARTUP_SEQUENCE_MAX_WAIT_TICKS);

  return (tx_return == TX_SUCCESS);
}

static void _control_shutdown_procedure ( void )
{
  rtc_alarm_struct alarm_settings;
  struct tm time_now;

  // Make sure everything is shut down
  shutdown_all_peripherals ();

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

  tx_thread_sleep (2);

  // Deinit all enabled peripherals
  shutdown_all_interfaces ();

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

  return battery_voltage;
}

static void _control_shut_down_all_peripherals ( void )
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

#warning "Make sure all peripherals are covered here."
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
//  HAL_NVIC_SetPriority (PVD_PVM_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ (PVD_PVM_IRQn);

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
  ULONG current_flags;
  UINT ret = TX_SUCCESS;
  // @formatter:off
  static bool gnss_complete = false,
              ct_complete = false,
              temperature_complete = false,
              light_complete = false,
              turbidity_complete = false,
              accelerometer_complete = false,
              waves_complete = false,
              iridium_complete = false;
                                                                                                          // @formatter:on
  bool iridium_ready = false;

  ct_complete = !controller_self->global_config->ct_enabled;
  temperature_complete = !controller_self->global_config->temperature_enabled;
  light_complete = !controller_self->global_config->light_enabled;
  turbidity_complete = !controller_self->global_config->turbidity_enabled;
  accelerometer_complete = !controller_self->global_config->accelerometer_enabled;

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
    if ( !ct_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->ct_thread);
    }

    if ( !temperature_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->temperature_thread);
    }
  }

  // When the GNSS thread is complete, we can run the Waves algo
  if ( current_flags & GNSS_THREAD_COMPLETED_SUCCESSFULLY )
  {
    gnss_complete = true;
    ret |= tx_thread_resume (controller_self->thread_handles->waves_thread);
  }

  // If GNSS thread errors out, do not run waves
  if ( current_flags & GNSS_THREAD_COMPLETED_WITH_ERRORS )
  {
    gnss_complete = true;

    if ( !ct_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->ct_thread);
    }

    if ( !temperature_complete )
    {
      ret |= tx_thread_resume (controller_self->thread_handles->temperature_thread);
    }
  }

  if ( (current_flags & CT_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & CT_THREAD_COMPLETED_WITH_ERRORS) )
  {
    ct_complete = true;
  }

  if ( (current_flags & TEMPERATURE_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & TEMPERATURE_THREAD_COMPLETED_WITH_ERRORS) )
  {
    temperature_complete = true;
  }

  if ( (current_flags & TURBIDITY_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & TURBIDITY_THREAD_COMPLETED_WITH_ERRORS) )
  {
    temperature_complete = true;
  }

  if ( (current_flags & LIGHT_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & LIGHT_THREAD_COMPLETED_WITH_ERRORS) )
  {
    light_complete = true;
  }

  if ( (current_flags & ACCELEROMETER_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & ACCELEROMETER_THREAD_COMPLETED_WITH_ERRORS) )
  {
    accelerometer_complete = true;
  }

  if ( (current_flags & WAVES_THREAD_COMPLETED_SUCCESSFULLY)
       | (current_flags & WAVES_THREAD_COMPLETED_WITH_ERRORS) )
  {
    waves_complete = true;
  }

  iridium_ready = gnss_complete && ct_complete && temperature_complete && light_complete
                  && turbidity_complete && accelerometer_complete && waves_complete;

  if ( iridium_ready )
  {
    controller_self->rf_switch.set_iridium_port ();
    ret |= tx_thread_resume (controller_self->thread_handles->iridium_thread);
  }

  iridium_complete = (current_flags & IRIDIUM_THREAD_COMPLETED_SUCCESSFULLY)
                     | (current_flags & IRIDIUM_THREAD_COMPLETED_WITH_ERRORS);

  if ( iridium_complete )
  {
    controller_self->shutdown_procedure ();
  }

  if ( ret != TX_SUCCESS )
  {
    controller_self->shutdown_all_pheripherals ();
    persistent_ram_deinit ();
    HAL_Delay (10);
    HAL_NVIC_SystemReset ();
  }
}

static void _control_monitor_and_handle_errors ( void )
{
  ULONG current_flags;
  ULONG gnss_errors, iridium_errors;

  // Get the error flags
  (void) tx_event_flags_get (controller_self->error_flags, ALL_EVENT_FLAGS, TX_OR_CLEAR,
                             &current_flags, TX_NO_WAIT);

  // Exit early case
  if ( current_flags == 0 )
  {
    return;
  }

  gnss_errors =
      current_flags
      & (GNSS_ERROR | GNSS_RESOLUTION_ERROR | GNSS_TOO_MANY_PARTIAL_MSGS
         | GNSS_SAMPLE_WINDOW_TIMEOUT | GNSS_FRAME_SYNC_FAILED | GNSS_SAMPLE_WINDOW_ERROR);
  iridium_errors = current_flags & (IRIDIUM_ERROR);

  if ( gnss_errors )
  {
    __handle_gnss_error (gnss_errors);
  }

  if ( iridium_errors )
  {
    __handle_iridium_error (iridium_errors);
  }

  if ( current_flags & RTC_ERROR )
  {
    __handle_rtc_error ();
  }

  if ( current_flags & CT_ERROR )
  {
    __handle_ct_error ();
  }

  if ( current_flags & TEMPERATURE_ERROR )
  {
    __handle_temperature_error ();
  }

  if ( current_flags & LIGHT_ERROR )
  {
    __handle_light_error ();
  }

  if ( current_flags & TURBIDITY_ERROR )
  {
    __handle_turbidity_error ();
  }

  if ( current_flags & ACCELEROMETER_ERROR )
  {
    __handle_accelerometer_error ();
  }

  if ( current_flags & WAVES_THREAD_ERROR )
  {
    __handle_waves_error ();
  }

  if ( current_flags & FILE_SYSTEM_ERROR )
  {
    __handle_file_system_error ();
  }

  if ( current_flags & MEMORY_CORRUPTION_ERROR )
  {
    controller_self->shutdown_all_pheripherals ();
    persistent_ram_deinit ();
    HAL_Delay (10);
    HAL_NVIC_SystemReset ();
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
  /*
   * TODO:
   *    [ ] Software reset the RTC
   *    [ ] Write the error to the error message buffer
   *    [ ] send the error message
   *    [ ] software reset
   */
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
static void __handle_ct_error ( void )
{
  /*
   * TODO:
   *    [ ] Shut down the CT sensor
   *    [ ] Set the temperature and salinity fields to error values
   *    [ ] Log the error in the error message buffer
   *    [ ] Set the CT_THREAD_SOMPLETED_WITH_ERRORS flag
   *    [ ] Terminate CT thread
   *    [ ] Continue application logic
   */
}
static void __handle_temperature_error ( void )
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
}
static void __handle_turbidity_error ( void )
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
static void __handle_light_error ( void )
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
static void __handle_accelerometer_error ( void )
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
  /*
   * TODO:
   *    [ ] First, ensure the error flag is set in the Waves thread somewhere...
   *    [ ] Set all SBD fields to error values
   *    [ ] Terminate waves thread
   *    [ ] Continue application logic
   */
}
static void __handle_iridium_error ( ULONG error_flags )
{
  /*
   * TODO:
   *    [ ] Power cycle modem
   *    [ ] Think about switching the on/off pin mode in case the wrong setting was input?
   *    [ ] Log the error in the error message buffer
   *    [ ] Controlled shut down
   *    [ ] System reset
   */
}
static void __handle_file_system_error ( void )
{
  /*
   * TODO:
   *    [ ] Unknown
   */
}

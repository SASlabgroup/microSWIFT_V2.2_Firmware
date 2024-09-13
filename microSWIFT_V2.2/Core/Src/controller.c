/*
 * controller.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include "controller.h"
#include "threadx_support.h"
#include "logger.h"

// @formatter:off
static Control  *self;

// Struct functions
static bool     _control_startup_procedure( void );
static bool     _control_all_threads_complete( void );
static real16_T _control_get_battery_voltage( void );
static void     _control_shut_down_all_peripherals ( void );
static void     _control_enter_processor_shutdown_mode ( void );
static void     _control_monitor_and_handle_errors ( void );

// Static helper functions

// @formatter:on

void controller_init ( Control *struct_ptr, microSWIFT_configuration *global_config,
                       Thread_Handles *thread_handles, TX_EVENT_FLAGS_GROUP *error_flags,
                       TX_EVENT_FLAGS_GROUP *init_flags, TX_EVENT_FLAGS_GROUP *irq_flags,
                       TX_EVENT_FLAGS_GROUP *complete_flags, TX_TIMER *timer,
                       ADC_HandleTypeDef *battery_adc_handle )
{
  // Init self
  self = struct_ptr;

  self->global_config = global_config;
  self->thread_handles = thread_handles;
  self->error_flags = error_flags;
  self->init_flags = init_flags;
  self->irq_flags = irq_flags;
  self->complete_flags = complete_flags;
  self->timer = timer;

  self->startup_procedure = _control_startup_procedure;
  self->all_threads_complete = _control_all_threads_complete;
  self->get_battery_voltage = _control_get_battery_voltage;
  self->shutdown_all_pheripherals = _control_shut_down_all_peripherals;
  self->enter_processor_shutdown_mode = _control_enter_processor_shutdown_mode;
  self->monitor_and_handle_errors = _control_monitor_and_handle_errors;

  // Init battery
  battery_init (&self->battery, battery_adc_handle, self->irq_flags);
  // Init RF switch
  rf_switch_init (&self->rf_switch);
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

  // Set the RF switch to GNSS port
  self->rf_switch.power_on ();
  self->rf_switch.set_gnss_port ();

  // Start core threads
  (void) tx_thread_resume (self->thread_handles->waves_thread);
  (void) tx_thread_resume (self->thread_handles->gnss_thread);
  (void) tx_thread_resume (self->thread_handles->iridium_thread);

  // Start optional component threads if enabled
  if ( self->global_config->ct_enabled )
  {
    init_success_flags |= CT_INIT_SUCCESS;
    (void) tx_thread_resume (self->thread_handles->ct_thread);
  }

  if ( self->global_config->temperature_enabled )
  {
    init_success_flags |= TEMPERATURE_INIT_SUCCESS;
    (void) tx_thread_resume (self->thread_handles->temperature_thread);
  }

  if ( self->global_config->light_enabled )
  {
    init_success_flags |= LIGHT_INIT_SUCCESS;
    (void) tx_thread_resume (self->thread_handles->light_thread);
  }

  if ( self->global_config->turbidity_enabled )
  {
    init_success_flags |= TURBIDITY_INIT_SUCCESS;
    (void) tx_thread_resume (self->thread_handles->turbidity_thread);
  }

  if ( self->global_config->accelerometer_enabled )
  {
    init_success_flags |= ACCELEROMETER_INIT_SUCCESS;
    (void) tx_thread_resume (self->thread_handles->accelerometer_thread);
  }

  // Flash power up sequence (this will also give threads time to execute their init procedures)
  led_sequence (INITIAL_LED_SEQUENCE);

  tx_return = tx_event_flags_get (self->init_flags, init_success_flags, TX_AND_CLEAR,
                                  &current_flags, STARTUP_SEQUENCE_MAX_WAIT_TICKS);

  return (tx_return == TX_SUCCESS);
}

static bool _control_all_threads_complete ( void )
{
  // Start with the core thread flags
  ULONG complete_flags = (GNSS_THREAD_COMPLETE | WAVES_THREAD_COMPLETE | IRIDIUM_THREAD_COMPLETE);
  ULONG dummy;

  if ( self->global_config->ct_enabled )
  {
    complete_flags |= CT_THREAD_COMPLETE;
  }

  if ( self->global_config->temperature_enabled )
  {
    complete_flags |= TEMPERATURE_THREAD_COMPLETE;
  }

  if ( self->global_config->turbidity_enabled )
  {
    complete_flags |= TURBIDITY_THREAD_COMPLETE;
  }

  if ( self->global_config->light_enabled )
  {
    complete_flags |= LIGHT_THREAD_COMPLETE;
  }

  if ( self->global_config->accelerometer_enabled )
  {
    complete_flags |= ACCELEROMETER_THREAD_COMPLETE;
  }

  return (tx_event_flags_get (self->complete_flags, complete_flags, TX_AND_CLEAR, &dummy,
  TX_NO_WAIT)
          == TX_SUCCESS);
}

static real16_T _control_get_battery_voltage ( void )
{
  real16_T battery_voltage =
    { BATTERY_ERROR_VOLTAGE_VALUE };
  battery_return_code_t ret = BATTERY_SUCCESS;

  ret = self->battery.get_voltage (&battery_voltage);
  if ( ret != BATTERY_SUCCESS )
  {
    switch ( ret )
    {
      case BATTERY_ADC_ERROR:
        uart_logger_log_line ("Battery ADC error, unable to obtain battery voltage");
        break;

      case BATTERY_TIMEOUT_ERROR:
        uart_logger_log_line ("Battery ADC timed out, unable to obtain battery voltage");
        break;

      default:
        uart_logger_log_line ("Unknown battery error, unable to obtain battery voltage");
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
}

static void _control_enter_processor_shutdown_mode ( void )
{
  // Retain all SRAM2 contents
  HAL_PWREx_EnableSRAM2ContentStandbyRetention (PWR_SRAM2_FULL_STANDBY);

#warning "Set an alarm in the RTC before entering shutdown mode!!!"
  // Ensure RTC is disabled
  CLEAR_BIT(RCC->BDCR, RCC_BDCR_RTCEN);

  // Clear the backup ram retention bit
  CLEAR_BIT(PWR->BDCR1, PWR_BDCR1_BREN);

  // Make sure the RTC INT_B pin is being pulled up (open drain on RTC)
  HAL_PWREx_EnableGPIOPullUp (PWR_GPIO_B, RTC_INT_B_Pin);

  // PWR_WAKEUP_PIN1_LOW_1 = PB2 --> RTC INT_B Low Polarity
  HAL_PWR_EnableWakeUpPin (PWR_WAKEUP_PIN1_LOW_1);

  // Clear the stop mode and standby mode flags
  SET_BIT(PWR->SR, PWR_SR_CSSF);

#ifndef DEBUG
  DBGMCU->CR = 0; // Disable debug, trace and IWDG in low-power modes
#endif

  /* Select Standby mode */
  MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_2);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  // Make sure register operations are complete
  (void) PWR->CR1;
  (void) SCB->SCR;

  // Enter low-power mode
  while ( 1 )
  {
    __DSB ();
    __WFI();
  }
}

static void _control_monitor_and_handle_errors ( void )
{
  ULONG current_flags;
  UINT tx_ret;

  (void) tx_event_flags_get (self->error_flags, 0, TX_OR_CLEAR, &current_flags, TX_NO_WAIT);
if (current_flags & )
}

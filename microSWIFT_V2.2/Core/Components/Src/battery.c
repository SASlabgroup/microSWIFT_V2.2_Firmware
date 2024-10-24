/*
 * Battery.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "battery.h"
#include "adc.h"
#include "stdbool.h"
// Object intance pointer
static Battery *self;

static battery_return_code_t battery_start_conversion ( void );
static battery_return_code_t battery_get_voltage ( real16_T *voltage );
static void battery_shutdown_adc ( void );

static bool adc_init_status = false;

void battery_init ( Battery *struct_ptr, ADC_HandleTypeDef *adc_handle,
                    TX_EVENT_FLAGS_GROUP *irq_flags )
{
  self = struct_ptr;
  self->adc_handle = adc_handle;
  self->irq_flags = irq_flags;
  self->voltage = 0;

  self->get_voltage = battery_get_voltage;
}

void battery_deinit ( void )
{
  if ( adc_init_status )
  {
    (void) HAL_ADC_Stop_IT (self->adc_handle);
    (void) HAL_ADC_DeInit (self->adc_handle);

    (void) HAL_SYSCFG_DisableVREFBUF ();

    (void) __HAL_RCC_VREF_CLK_DISABLE();

    (void) HAL_PWREx_DisableVddA ();

    adc_init_status = false;
  }
}

static battery_return_code_t battery_start_conversion ( void )
{
  battery_return_code_t return_code;

  if ( !adc_init_status )
  {
    if ( !adc1_init () )
    {
      return BATTERY_ADC_ERROR;
    }

    adc_init_status = true;
  }

  /** Enable the VREF clock
   */
  __HAL_RCC_VREF_CLK_ENABLE();

  /** Configure the internal voltage reference buffer voltage scale
   */
  HAL_SYSCFG_VREFBUF_VoltageScalingConfig (SYSCFG_VREFBUF_VOLTAGE_SCALE1);

  /** Enable the Internal Voltage Reference buffer
   */
  HAL_SYSCFG_EnableVREFBUF ();

  /** Configure the internal voltage reference buffer high impedance mode
   */
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig (SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

  // Enable VDDA, power supply to ADC
  HAL_PWREx_EnableVddA ();

  tx_thread_sleep (1);

  if ( HAL_ADCEx_Calibration_Start (self->adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)
       != HAL_OK )
  {
    return_code = BATTERY_ADC_ERROR;
    return return_code;
  }
  // Need at least a 4 ADC clock cycle delay after calibration before doing anything else with the
  // ADC -- 10ms ought to do.
  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
  self->calibration_offset = HAL_ADCEx_Calibration_GetValue (self->adc_handle, ADC_SINGLE_ENDED);

  if ( HAL_ADC_Start_IT (self->adc_handle) != HAL_OK )
  {
    return_code = BATTERY_ADC_ERROR;
    return return_code;
  }

  return_code = BATTERY_SUCCESS;
  return return_code;
}

static battery_return_code_t battery_get_voltage ( real16_T *voltage )
{
  uint32_t max_ticks_to_get_voltage = TX_TIMER_TICKS_PER_SECOND;
  ULONG actual_flags;
  battery_return_code_t ret;

  ret = battery_start_conversion ();
  if ( ret != BATTERY_SUCCESS )
  {
    voltage->bitPattern = BATTERY_ERROR_VOLTAGE_VALUE;
    return ret;
  }

  // If it either takes too long or an EDC error is detected, set the battery voltage to the error value
  if ( (tx_event_flags_get (self->irq_flags, BATTERY_CONVERSION_COMPLETE, TX_OR_CLEAR,
                            &actual_flags, max_ticks_to_get_voltage)
        != TX_SUCCESS) )
  {
    voltage->bitPattern = BATTERY_ERROR_VOLTAGE_VALUE;
    return BATTERY_TIMEOUT_ERROR;
  }

  *voltage = floatToHalf (self->voltage);
  return BATTERY_SUCCESS;
}

static void battery_shutdown_adc ( void )
{
  (void) HAL_ADC_Stop_IT (self->adc_handle);
  (void) HAL_ADC_DeInit (self->adc_handle);

  (void) HAL_SYSCFG_DisableVREFBUF ();

  (void) __HAL_RCC_VREF_CLK_DISABLE();

  (void) HAL_PWREx_DisableVddA ();
}

void battery_set_voltage ( float voltage )
{
  self->voltage = voltage;
}


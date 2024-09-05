/*
 * Battery.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include "stm32u5xx_hal.h"
#include "tx_api.h"
#include "NEDWaves/rtwhalf.h"
#include "app_threadx.h"

#define NUMBER_OF_ADC_SAMPLES 400
#define BATTERY_CALIBRATION_OFFSET 0
#define MICROVOLTS_PER_VOLT 1000000
#define ADC_CALIBRATION_CONSTANT_MICROVOLTS 25000
#define BATTERY_ERROR_VOLTAGE_VALUE 0x70E2
#define ADC_MICROVOLTS_PER_BIT 3292

// @formatter:off
typedef enum battery_error_code
{
  BATTERY_SUCCESS               = 0,
  BATTERY_CONVERSION_ERROR      = -1,
  BATTERY_TIMEOUT_ERROR         = -2,
  BATTERY_ADC_ERROR             = -3
} battery_error_code_t;

typedef struct Battery
{
  ADC_HandleTypeDef     *adc_handle;
  TX_EVENT_FLAGS_GROUP  *irq_flags;

  float                 voltage;
  uint32_t              calibration_offset;

  battery_error_code_t  (*get_voltage) ( real16_T *voltage );
} Battery;

void battery_init ( ADC_HandleTypeDef *adc_handle, TX_EVENT_FLAGS_GROUP *irq_flags );
void battery_set_voltage (float voltage);

// @formatter:on

#endif /* SRC_BATTERY_H_ */

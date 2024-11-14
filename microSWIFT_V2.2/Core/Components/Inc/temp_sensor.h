/*
 * temp_sensor.h
 *
 * !!! MOSTLY COPPIED FROM BLUE ROBOTICS !!!
 *
 * https://github.com/bluerobotics/BlueRobotics_TSYS01_Library?tab=readme-ov-file
 *
 *  Created on: Feb 9, 2024
 *      Author: Phil
 */

#ifndef INC_PERIPHERALS_TEMP_SENSOR_H_
#define INC_PERIPHERALS_TEMP_SENSOR_H_

#include "stdbool.h"
#include "stddef.h"
#include "tx_api.h"
#include "stm32u5xx_hal.h"
#include "gpio.h"
#include "configuration.h"
#include "microSWIFT_return_codes.h"

// @formatter:off

#define TSYS01_ADDR                       	(0x77 << 1)
#define TSYS01_RESET                      	(0x1E)
#define TSYS01_ADC_READ                    	(0x00)
#define TSYS01_ADC_TEMP_CONV               	(0x48)
#define TSYS01_PROM_READ                   	(0XA0)

#define TEMPERATURE_VALUES_ERROR_CODE		(0X70E2)

#define TEMPERATURE_SENSOR_I2C_TIMEOUT      2

typedef struct Temperature
{
  microSWIFT_configuration  *global_config;
  TX_EVENT_FLAGS_GROUP      *error_flags;
  I2C_HandleTypeDef         *i2c_handle;
  TX_TIMER                  *timer;

  bool                      timer_timeout;

  uSWIFT_return_code_t      (*self_test) ( float *optional_reading );
  uSWIFT_return_code_t      (*get_readings) ( bool get_single_reading, float *temperature );
  uSWIFT_return_code_t      (*start_timer) ( uint16_t timeout_in_minutes );
  uSWIFT_return_code_t      (*stop_timer) ( void );
  void                      (*on) ( void );
  void                      (*off) ( void );

  float                     converted_temp;
  float                     C[8]; // Cal data array
  uint32_t                  D1; // Read data (unconverted temp)
  uint32_t                  adc;

  gpio_pin_struct           pwr_gpio;
} Temperature;

void temperature_init ( Temperature *struct_ptr, microSWIFT_configuration *global_config,
                        I2C_HandleTypeDef *i2c_handle, TX_EVENT_FLAGS_GROUP *error_flags,
                        TX_TIMER *timer, TX_MUTEX *i2c_mutex, bool clear_calibration_data );
void temperature_deinit ( void );
void temperature_timer_expired ( ULONG expiration_input );
bool temperature_get_timeout_status ( void );

// @formatter:on
#endif /* INC_PERIPHERALS_TEMP_SENSOR_H_ */

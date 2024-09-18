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

// @formatter:off

#define TSYS01_ADDR                       	(0x77 << 1)
#define TSYS01_RESET                      	(0x1E)
#define TSYS01_ADC_READ                    	(0x00)
#define TSYS01_ADC_TEMP_CONV               	(0x48)
#define TSYS01_PROM_READ                   	(0XA0)

#define TEMPERATURE_AVERAGED_ERROR_CODE		(0X70E2)

typedef enum tmperature_error_code
{
  TEMPERATURE_SUCCESS = 0,
  TEMPERATURE_CONVERSION_ERROR = -1,
  TEMPERATURE_TIMEOUT_ERROR = -2,
  TEMPERATURE_COMMUNICATION_ERROR = -3
} temperature_return_code_t;

typedef struct Temperature
{
  microSWIFT_configuration  *global_config;
  I2C_HandleTypeDef         *i2c_handle;
  TX_EVENT_FLAGS_GROUP      *error_flags;
  TX_MUTEX                  *i2c_mutex;

  temperature_return_code_t (*self_test) ( float *optional_reading );
  temperature_return_code_t (*get_readings) ( bool get_single_reading, float *temperature );
  void                      (*on) ( void );
  void                      (*off) ( void );

  float                     converted_temp;
  float                     C[8]; // Cal data array
  uint32_t                  D1; // Read data (unconverted temp)
  uint32_t                  adc;

  gpio_pin_struct           pwr_gpio;
} Temperature;

void temperature_init ( microSWIFT_configuration *global_config, Temperature *struct_ptr,
                        I2C_HandleTypeDef *i2c_handle, TX_EVENT_FLAGS_GROUP *error_flags,
                        TX_MUTEX *i2c_mutex, bool clear_calibration_data );
void temperature_deinit ( void );

// @formatter:on
#endif /* INC_PERIPHERALS_TEMP_SENSOR_H_ */

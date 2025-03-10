/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "stdbool.h"
#include "stdint.h"

typedef struct
{
  uint8_t major_rev :4;
  uint8_t minor_rev :4;
} microSWIFT_firmware_version_t;

typedef struct __attribute__((packed)) microSWIFT_configuration
{
  uint32_t tracking_number;
  uint32_t gnss_samples_per_window;
  uint32_t duty_cycle;
  uint32_t iridium_max_transmit_time;
  uint32_t gnss_max_acquisition_wait_time;
  uint32_t gnss_sampling_rate;
  uint32_t total_ct_samples;
  uint32_t total_temp_samples;
  uint32_t total_light_samples;
  uint32_t total_turbidity_samples;

  bool iridium_v3f;
  bool gnss_high_performance_mode;
  bool ct_enabled;
  bool temperature_enabled;
  bool light_enabled;
  bool turbidity_enabled;

  const microSWIFT_firmware_version_t firmware_version;

  const char compile_date_flash[11];
  const char compile_time_flash[9];
} microSWIFT_configuration;

#endif /* INC_CONFIGURATION_H_ */

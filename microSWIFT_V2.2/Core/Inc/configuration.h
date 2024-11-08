/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/*
 * Debugging settings
 */

// For testing and debugging with a very short sample window
//#define DEBUGGING_FAST_CYCLE
// If a 1 min sleep window is desired
//#define SHORT_SLEEP
/*
 * Configuration options
 */

// For debugging, redefine sample window parameters to be faster
#ifdef DEBUGGING_FAST_CYCLE

#define TOTAL_SAMPLES_PER_WINDOW 1024
#define IRIDIUM_MAX_TRANSMIT_TIME 10
#define GNSS_MAX_ACQUISITION_WAIT_TIME 10
#define SAMPLE_WINDOWS_PER_HOUR 1
#define WATCHDOG_PERIOD 600000 // 1 min (in ms)
#else
// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 1024

// The max time in MINUTES to try to get an Iridium message off
#define IRIDIUM_MAX_TRANSMIT_TIME 1

// The version of RckBlock 9603 modem
#define IRIDIUM_V3F false

// The max time in MINUTES without good data from GNSS before commanding to sleep
// !! Must be greater than 0
// **** In the case of SAMPLE_WINDOWS_PER_HOUR > 1, this will only apply to the very
//      first sample window. Subsequent windows will calculate the GNSS acq time
#define GNSS_MAX_ACQUISITION_WAIT_TIME 2

#warning "Convert this to duty cycle, where the input is the number of minutes for a full cycle."
// Are we doing 1 or two sample windows per hour
#define SAMPLE_WINDOWS_PER_HOUR 2

// Mins for a full sample, process, transmit period
#define DUTY_CYCLE_PERIOD 30

#ifdef DEBUG
#define WATCHDOG_PERIOD 600000 // 1 min (in ms)
#else
#define WATCHDOG_PERIOD 60000 // 1 min (in ms)
#endif
#endif // DEBUGGING_FAST_CYCLE

// Sampling rate in Hz for the GNSS sensor
// !! Must be either 4 or 5
#define GNSS_SAMPLING_RATE 4

// Determine whether or not the GNSS sensor should be set to high performance mode
#define GNSS_HIGH_PERFORMANCE_MODE_ENABLED false

// Buffer time in mins added to the GNSS sample window timer
// Ex: If TOTAL_SAMPLES_PER_WINDOW = 4096 and GNSS_SAMPLING_RATE = 4, sample window will take 17 mins to complete.
//     The value defined for GNSS_WINDOW_BUFFER_TIME is added to this, so if the GNSS sample window does not complete
//     (17 + GNSS_WINDOW_BUFFER_TIME) mins after receiving first sample, then it times out.
#define GNSS_WINDOW_BUFFER_TIME 2

// If there is a CT sensor present
#define CT_ENABLED false
#define TOTAL_CT_SAMPLES 10

// If there is a Blue Robotics I2C temperature sensor presen
#define TEMPERATURE_ENABLED false
#define TOTAL_TEMPERATURE_SAMPLES 10

// If there is a light sensor present
#define LIGHT_SENSOR_ENABLED true
#define TOTAL_LIGHT_SAMPLES (TOTAL_SAMPLES_PER_WINDOW / GNSS_SAMPLING_RATE)

// If there is a turbidity sensor present
#define TURBIDITY_SENSOR_ENABLED false
#define TOTAL_TURBIDITY_SAMPLES (TOTAL_SAMPLES_PER_WINDOW / GNSS_SAMPLING_RATE)

// If there is a fast accelerometer present
#define ACCELEROMETER_ENABLED false

typedef struct microSWIFT_configuration
{
  uint32_t samples_per_window;
  uint32_t windows_per_hour;
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
  bool accelerometer_enabled;

} microSWIFT_configuration;

#endif /* INC_CONFIGURATION_H_ */

/*
 * pcf2131_reg.c
 *
 *  Created on: Jul 24, 2024
 *      Author: philbush
 */

#include "pcf2131_reg.h"

uint8_t dec_to_bcd[100] =
  { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, // 0-9
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, // 10-19
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, // 20-29
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, // 30-39
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, // 40-49
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, // 50-59
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, // 60-69
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, // 70-79
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, // 80-89
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99  // 90-99
    };

uint8_t bcd_to_dec[256] =
  {
  // 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0,
    1, 2, 3, 4, 5, 6, 7,
    // 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    8, 9, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    10, 11, 12, 13, 14, 15, 16, 17,
    // 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    18, 19, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    20, 21, 22, 23, 24, 25, 26, 27,
    // 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    28, 29, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    30, 31, 32, 33, 34, 35, 36, 37,
    // 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    38, 39, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    40, 41, 42, 43, 44, 45, 46, 47,
    // 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    48, 49, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
    50, 51, 52, 53, 54, 55, 56, 57,
    // 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
    58, 59, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
    60, 61, 62, 63, 64, 65, 66, 67,
    // 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
    68, 69, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
    70, 71, 72, 73, 74, 75, 76, 77,
    // 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    78, 79, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    80, 81, 82, 83, 84, 85, 86, 87,
    // 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    88, 89, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
    90, 91, 92, 93, 94, 95, 96, 97,
    // 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
    98, 99, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7,
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR,
    // 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
    BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR, BCD_ERROR };

static uint8_t weekday_from_date ( int y, int m, int d );

int32_t pcf2131_register_io_functions ( dev_ctx_t *dev_handle, dev_init_ptr init_fn,
                                        dev_deinit_ptr deinit_fn, dev_write_ptr bus_write_fn,
                                        dev_read_ptr bus_read_fn, void *optional_handle )
{
  dev_handle->init = init_fn;
  dev_handle->deinit = deinit_fn;
  dev_handle->bus_read = bus_read_fn;
  dev_handle->bus_write = bus_write_fn;
  dev_handle->handle = optional_handle;

  return dev_handle->init ();
}

int32_t pcf2131_set_date_time ( dev_ctx_t *dev_handle, struct tm *input_date_time )
{
  int32_t ret = PCF2131_OK;
  uint8_t bcd_year = dec_to_bcd[input_date_time->tm_year];
  uint8_t bcd_month = dec_to_bcd[input_date_time->tm_mon];
  uint8_t weekday =
      (input_date_time->tm_wday == WEEKDAY_UNKNOWN) ?
          weekday_from_date (input_date_time->tm_year, input_date_time->tm_mon,
                             input_date_time->tm_mday) :
          (uint8_t) input_date_time->tm_wday;
  uint8_t bcd_day = dec_to_bcd[input_date_time->tm_mday];
  uint8_t bcd_hour = dec_to_bcd[input_date_time->tm_hour];
  uint8_t bcd_min = dec_to_bcd[input_date_time->tm_min];
  uint8_t bcd_sec = dec_to_bcd[input_date_time->tm_sec];
  uint8_t bcd_sec_fraction = 0x00;
  uint8_t rtc_date_time[8] =
    { bcd_sec_fraction, bcd_sec, bcd_min, bcd_hour, bcd_day, weekday, bcd_month, bcd_year };

  /*
   • set STOP bit
   • set CPR (register SR_RESET, CPR is logic 1)
   • address counter rolls over to address 06h
   • set time (100th seconds, seconds to years)
   • wait for external time reference to indicate that time counting should start
   • clear STOP bit (time starts counting from now)
   */

  ret = pcf2131_set_stop_bit (dev_handle);

  ret |= pcf2131_clear_prescalar (dev_handle);

  ret |= dev_handle->bus_write (NULL, 0, ONE_100_SEC_REG_ADDR, &(rtc_date_time[0]),
                                sizeof(rtc_date_time));

  ret |= pcf2131_clear_stop_bit (dev_handle);

  return ret;
}

int32_t pcf2131_get_date_time ( dev_ctx_t *dev_handle, struct tm *return_date_time )
{
// TODO: need to ensure sequential read is happening in this function
  int32_t ret = PCF2131_OK;
  pcf2131_reg_t time_date[8] =
    { 0 };

  ret = dev_handle->bus_read (NULL, 0, ONE_100_SEC_REG_ADDR, (uint8_t*) &(time_date[0]),
                              sizeof(time_date));

  return_date_time->tm_sec = bcd_to_dec[time_date[1].seconds.tens_place
                                        | time_date[1].seconds.units_place];
  return_date_time->tm_min = bcd_to_dec[time_date[2].minutes.tens_place
                                        | time_date[2].minutes.units_place];
  return_date_time->tm_hour = bcd_to_dec[time_date[3].hours.format_24hr.hours_tens_place
                                         | time_date[3].hours.format_24hr.hours_units_place];
  return_date_time->tm_mday = bcd_to_dec[time_date[4].days.tens_place
                                         | time_date[4].days.units_place];
  return_date_time->tm_wday = time_date[5].weekday.weekday;
  return_date_time->tm_mon = bcd_to_dec[time_date[6].months.month];
  return_date_time->tm_year = bcd_to_dec[time_date[7].years.tens_place
                                         | time_date[7].years.units_place];

  return ret;
}

int32_t pcf2131_set_alarm ( dev_ctx_t *dev_handle, rtc_alarm_struct *alarm_setting )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_alarm ( dev_ctx_t *dev_handle, rtc_alarm_struct *return_alarm_setting )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_config_int_a ( dev_ctx_t *dev_handle, pcf2131_irq_config_struct *irq_config )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_config_int_b ( dev_ctx_t *dev_handle, pcf2131_irq_config_struct *irq_config )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_config_int_signal_behavior ( dev_ctx_t *dev_handle, int_signal_behavior_t behavior )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_timestamp ( dev_ctx_t *dev_handle, pcf2131_timestamp_t which_timestamp,
                                struct tm *return_date_time )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_temp_comp_config ( dev_ctx_t *dev_handle, bool en )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_set_stop_bit ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_stop_bit ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_one_100_sec_counter_config ( dev_ctx_t *dev_handle, bool en )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_hours_format_config ( dev_ctx_t *dev_handle, hours_ampm_bit_t hours_format )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_msf_flag ( dev_ctx_t *dev_handle, bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_msf_flag ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_watchdog_flag ( dev_ctx_t *dev_handle, bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_watchdog_flag ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_alarm_flag ( dev_ctx_t *dev_handle, bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_alarm_flag ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_battery_switch_over_flag ( dev_ctx_t *dev_handle, bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_battery_switch_over_flag ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_config_timestamp_flag ( dev_ctx_t *dev_handle, uint8_t which_timestamp, bool en )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_timestamp_flag ( dev_ctx_t *dev_handle, uint8_t which_timestamp,
bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_timestamp_flag ( dev_ctx_t *dev_handle, uint8_t which_timestamp )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_set_temp_meas_period ( dev_ctx_t *dev_handle, temp_meas_period_t meas_period )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_set_clkout_freq ( dev_ctx_t *dev_handle, clock_frequency_t freq_out )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_perform_otp_refresh ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_set_crystal_aging_offset ( dev_ctx_t *dev_handle, aging_offset_t offset )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_config_pwr_mgmt_scheme ( dev_ctx_t *dev_handle, pwr_mgmt_t mgmt_scheme )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_osf_flag ( dev_ctx_t *dev_handle, bool *return_flag )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_software_reset ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_prescalar ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_timestamps ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_clear_prescalar_and_timestamps ( dev_ctx_t *dev_handle )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_watchdog_irq_config ( dev_ctx_t *dev_handle, bool enable )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_watchdog_config_time_source ( dev_ctx_t *dev_handle,
                                              watchdog_time_source_t time_source )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_set_watchdog_timer_value ( dev_ctx_t *dev_handle, uint8_t timer_value )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

static uint8_t weekday_from_date ( int y, int m, int d )
{
  /* wikipedia.org/wiki/Determination_of_the_day_of_the_week#Implementation-dependent_methods */
  uint8_t weekday = (uint8_t) ((d += m < 3 ?
      y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400)
                               % 7);

  return weekday;
}

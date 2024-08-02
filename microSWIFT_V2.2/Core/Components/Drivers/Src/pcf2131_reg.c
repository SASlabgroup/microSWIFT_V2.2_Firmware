/*
 * pcf2131_reg.c
 *
 *  Created on: Jul 24, 2024
 *      Author: philbush
 */

#include "pcf2131_reg.h"

static uint8_t weekday_from_date ( int y, int m, int d );

int32_t pcf2131_register_io_functions ( dev_ctx_t *dev_handle, dev_write_ptr bus_write_fn,
                                        dev_read_ptr bus_read_fn, void *optional_handle )
{
  dev_handle->bus_read = bus_read_fn;
  dev_handle->bus_write = bus_write_fn;
  dev_handle->handle = optional_handle;

  return PCF2131_OK;
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

int32_t pcf2131_set_alarm ( dev_ctx_t *dev_handle, pcf2131_alarm_struct *alarm_setting )
{
  int32_t ret = PCF2131_OK;

  return ret;
}

int32_t pcf2131_get_alarm ( dev_ctx_t *dev_handle, pcf2131_alarm_struct *return_alarm_setting )
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

int32_t pcf2131_watchdog_irq_signal_config ( dev_ctx_t *dev_handle,
                                             watchdog_int_signal_t signal_config )
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

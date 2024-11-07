/*
 * ext_rtc.c
 *
 *  Created on: Jul 29, 2024
 *      Author: philbush
 */

#include "ext_rtc.h"
#include "spi.h"
#include "gpio.h"
#include "app_threadx.h"
#include "main.h"
#include "threadx_support.h"

#warning"move the RTC dev kit off the Nucleo so pinout will be more free."
#warning"try piping the clock signal into the Nucleo"

static Ext_RTC *rtc_self;

/* Core struct functions */
static rtc_return_code _ext_rtc_setup_rtc ( void );
static rtc_return_code _ext_rtc_config_watchdog ( uint32_t period_ms );
static rtc_return_code _ext_rtc_refresh_watchdog ( void );
static rtc_return_code _ext_rtc_set_date_time ( struct tm *input_date_time );
static rtc_return_code _ext_rtc_get_date_time ( struct tm *return_date_time );
static rtc_return_code _ext_rtc_set_timestamp ( pcf2131_timestamp_t which_timestamp );
static rtc_return_code _ext_rtc_get_timestamp ( pcf2131_timestamp_t which_timestamp,
                                                time_t *return_timestamp );
static rtc_return_code _ext_rtc_set_alarm ( rtc_alarm_struct alarm_setting );
static rtc_return_code _ext_rtc_clear_flag ( rtc_flag_t which_flag );

/* SPI driver functions */
static int32_t _ext_rtc_spi_init ( void );
static int32_t _ext_rtc_spi_deinit ( void );
static int32_t _ext_rtc_read_reg_spi ( void *unused_handle, uint16_t unused_bus_address,
                                       uint16_t reg_address, uint8_t *read_data,
                                       uint16_t data_length );
static int32_t _ext_rtc_write_reg_spi ( void *unused_handle, uint16_t unused_bus_address,
                                        uint16_t reg_address, uint8_t *write_data,
                                        uint16_t data_length );
static void _ext_rtc_ms_delay ( uint32_t delay );
static uint8_t __weekday_from_date ( int y, int m, int d );
//static int32_t _ext_rtc_read_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
//                                           uint16_t reg_address, uint8_t *read_data,
//                                           uint16_t data_length );
//static int32_t _ext_rtc_write_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
//                                            uint16_t reg_address, uint8_t *write_data,
//                                            uint16_t data_length );
/**
 * @brief  Initialize the Ext_RTC struct
 *
 * @note   INT_A pin will be used for the watchdog, INT_B pin will be used for the Alarm function.
 *
 * @param  struct_ptr:= Global struct pointer, saved locally as static pointer
 * @param  rtc_spi_bus:= Handle for SPI bus
 * @param  messaging_queue:= Pointer to global messaging queue for inbound requests
 * @retval rtc_return_code
 */
rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
                               TX_SEMAPHORE *rtc_spi_sema )
{
  int32_t ret;
  uint8_t register_read = 0;
  // Grab the global struct pointer
  rtc_self = struct_ptr;

  rtc_self->rtc_spi_bus = rtc_spi_bus;

  rtc_self->spi_sema = rtc_spi_sema;

  rtc_self->int_a_pin.port = RTC_INT_A_GPIO_Port;
  rtc_self->int_a_pin.pin = RTC_INT_A_Pin;
  rtc_self->int_b_pin.port = RTC_INT_B_GPIO_Port;
  rtc_self->int_b_pin.pin = RTC_INT_B_Pin;

  rtc_self->ts_pins[0].port = RTC_TIMESTAMP_1_GPIO_Port;
  rtc_self->ts_pins[0].pin = RTC_TIMESTAMP_1_Pin;

  rtc_self->ts_pins[1].port = RTC_TIMESTAMP_2_GPIO_Port;
  rtc_self->ts_pins[1].pin = RTC_TIMESTAMP_2_Pin;

  rtc_self->ts_pins[2].port = RTC_TIMESTAMP_3_GPIO_Port;
  rtc_self->ts_pins[2].pin = RTC_TIMESTAMP_3_Pin;

  rtc_self->ts_pins[3].port = RTC_TIMESTAMP_4_GPIO_Port;
  rtc_self->ts_pins[3].pin = RTC_TIMESTAMP_4_Pin;

  rtc_self->ts_in_use[0] = false;
  rtc_self->ts_in_use[1] = false;
  rtc_self->ts_in_use[2] = false;
  rtc_self->ts_in_use[3] = false;

  rtc_self->watchdog_refresh_time_val = 0;

  // Interrupt configuration
  rtc_self->irq_config.sec_irq_en = false;
  rtc_self->irq_config.min_irq_en = false;
  rtc_self->irq_config.sec_min_pulsed_irq_en = false;
  rtc_self->irq_config.watchdog_irq_en = true;
  rtc_self->irq_config.alarm_irq_en = true;
  rtc_self->irq_config.batt_flag_irq_en = false;
  rtc_self->irq_config.batt_low_irq_en = false;
  rtc_self->irq_config.timestamp_1_irq_en = false;
  rtc_self->irq_config.timestamp_2_irq_en = false;
  rtc_self->irq_config.timestamp_3_irq_en = false;
  rtc_self->irq_config.timestamp_4_irq_en = false;
  // If a mask bit is set, the associated IRQ is masked and will not fire.
  // Int A will be used for the Alarm
  *((uint8_t*) &rtc_self->irq_config.int_a_mask_1) = 0xFF;
  rtc_self->irq_config.int_a_mask_1.alarm_irq_mask = false;
  rtc_self->irq_config.int_a_mask_1.dash_bit = 0b00;
  *((uint8_t*) &rtc_self->irq_config.int_a_mask_2) = 0xFF;
  rtc_self->irq_config.int_a_mask_2.dash_bit = 0b0000;
  // Int B will be used for Watchdog
  *((uint8_t*) &rtc_self->irq_config.int_b_mask_1) = 0xFF;
  rtc_self->irq_config.int_b_mask_1.watchdog_irq_mask = false;
  rtc_self->irq_config.int_b_mask_1.dash_bit = 0b00;
  *((uint8_t*) &rtc_self->irq_config.int_b_mask_2) = 0xFF;
  rtc_self->irq_config.int_b_mask_2.dash_bit = 0b0000;

  rtc_self->setup_rtc = _ext_rtc_setup_rtc;
  rtc_self->config_watchdog = _ext_rtc_config_watchdog;
  rtc_self->refresh_watchdog = _ext_rtc_refresh_watchdog;
  rtc_self->set_date_time = _ext_rtc_set_date_time;
  rtc_self->get_date_time = _ext_rtc_get_date_time;
  rtc_self->set_timestamp = _ext_rtc_set_timestamp;
  rtc_self->get_timestamp = _ext_rtc_get_timestamp;
  rtc_self->set_alarm = _ext_rtc_set_alarm;
  rtc_self->clear_flag = _ext_rtc_clear_flag;

  // Register dev_ctx functions
  if ( pcf2131_register_io_functions (&rtc_self->dev_ctx, _ext_rtc_spi_init, _ext_rtc_spi_deinit,
                                      _ext_rtc_write_reg_spi, _ext_rtc_read_reg_spi,
                                      _ext_rtc_ms_delay, NULL)
       != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Software reset the RTC
  if ( pcf2131_software_reset (&rtc_self->dev_ctx) != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // read the Software Reset register to see if the bit pattern matches default
  ret = rtc_self->dev_ctx.bus_read (NULL, 0, RESET_REG_ADDR, &register_read, 1);

  if ( ret != PCF2131_OK || register_read != RESET_REG_RESET_VAL )
  {
    return RTC_SPI_ERROR;
  }

  ret = rtc_self->clear_flag (ALL_RTC_FLAGS);

  return ret;
}

/**
 * @brief  Apply all required RTC settings, with the exception of the watchdog timer.
 * @param  None
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_setup_rtc ( void )
{
  int32_t ret = RTC_SUCCESS;

  // Clock output
  ret = pcf2131_set_clkout_freq (&(rtc_self->dev_ctx), FREQ_32768);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Power management scheme
  ret = pcf2131_config_pwr_mgmt_scheme (&(rtc_self->dev_ctx), SWITCH_OVER_DIS_LOW_BATT_DIS);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Temperature measurement/ compensation
  ret = pcf2131_temp_comp_config (&(rtc_self->dev_ctx), true);
  ret |= pcf2131_set_temp_meas_period (&(rtc_self->dev_ctx), EVERY_32_MINS);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Interrupts: Int A will be used for alarm, Int B for watchdog
  ret = pcf2131_config_interrupts (&(rtc_self->dev_ctx), &rtc_self->irq_config);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Timestamps
  for ( int i = 0; i < NUMBER_OF_TIMESTAMPS; i++ )
  {
    ret |= pcf2131_set_timestamp_enable (&rtc_self->dev_ctx, (pcf2131_timestamp_t) i, true);
    ret |= pcf2131_set_timestamp_store_option (&rtc_self->dev_ctx, (pcf2131_timestamp_t) i,
                                               FIRST_EVENT_STORED);
  }

  ret |= pcf2131_clear_timestamps (&rtc_self->dev_ctx);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Perform OTP refresh, only on first start
  if ( is_first_sample_window () )
  {
    ret = pcf2131_perform_otp_refresh (&rtc_self->dev_ctx);
    if ( ret != PCF2131_OK )
    {
      return RTC_SPI_ERROR;
    }
  }

  // Clear POR bit -- crystal should be plenty good by now
  ret = pcf2131_por_config (&rtc_self->dev_ctx, false);

  return ret;
}

/**
 * @brief  Configure the watchdog, hook it up to INT pin, set the timer.
 *         !!! The watchdog will start after this function returns success !!!
 * @param  period_ms:= Watchdog refresh interval in milliseconds
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_config_watchdog ( uint32_t period_ms )
{
  int32_t ret = RTC_SUCCESS;
  watchdog_time_source_t clock_select;
  pcf2131_irq_config_struct irq_config =
    { 0 };

  // We'll establish a minimum refresh interval of 10 seconds
  if ( (period_ms < RTC_WATCHDOG_MIN_REFRESH) || (period_ms > PCF2131_1_64HZ_CLK_MAX_PERIOD_MS) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  // Figure out what watchdog clock rate to use
  // Reference Table 59 in datasheet
  if ( period_ms > PCF2131_1_4HZ_CLK_MAX_PERIOD_MS )
  {
    clock_select = HZ_1_64;
    rtc_self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * (1.0f / 64.0f)) + 1));
  }
  else if ( period_ms > (PCF2131_4HZ_CLK_MAX_PERIOD_MS) )
  {
    clock_select = HZ_1_4;
    rtc_self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * (1.0f / 4.0f)) + 1));
  }
  else if ( period_ms > PCF2131_64HZ_CLK_MAX_PERIOD_MS )
  {
    clock_select = HZ_4;
    rtc_self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * 4.0f) + 1));
  }
  else
  {
    clock_select = HZ_64;
    rtc_self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * 64.0f) + 1));
  }

  ret = pcf2131_watchdog_config_time_source (&rtc_self->dev_ctx, clock_select);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Set the watchdog timer value -- watchdog will start at this point
  ret = pcf2131_set_watchdog_timer_value (&rtc_self->dev_ctx, rtc_self->watchdog_refresh_time_val);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief  Refresh the watchdog.
 * @param  None
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_refresh_watchdog ( void )
{
  int32_t ret = RTC_SUCCESS;

  ret = pcf2131_set_watchdog_timer_value (&rtc_self->dev_ctx, rtc_self->watchdog_refresh_time_val);
  if ( ret != PCF2131_OK )
  {
    ret = RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief  Set the RTC date and Timer registers
 * @param  input_date_time:= struct tm containing the desired time settings
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_set_date_time ( struct tm *input_date_time )
{
  int32_t ret = RTC_SUCCESS;

  input_date_time->tm_wday =
      (input_date_time->tm_wday == WEEKDAY_UNKNOWN) ?
          __weekday_from_date (input_date_time->tm_year, input_date_time->tm_mon,
                               input_date_time->tm_mday) :
          (uint8_t) input_date_time->tm_wday;

  // Convert to BCD format
  struct_tm_dec_to_bcd (input_date_time);

  if ( (input_date_time->tm_sec == BCD_ERROR) || (input_date_time->tm_min == BCD_ERROR)
       || (input_date_time->tm_hour == BCD_ERROR) || (input_date_time->tm_mday == BCD_ERROR)
       || (input_date_time->tm_mon == BCD_ERROR) || (input_date_time->tm_year == BCD_ERROR)
       || (input_date_time->tm_year == BCD_ERROR) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  ret = pcf2131_set_date_time (&rtc_self->dev_ctx, input_date_time);

  if ( ret != PCF2131_OK )
  {
    ret = RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the current date/time from the RTC
 * @param  return_date_time:= Return pointer for struct tm
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_get_date_time ( struct tm *return_date_time )
{
  int32_t ret = RTC_SUCCESS;

  ret = pcf2131_get_date_time (&rtc_self->dev_ctx, return_date_time);

  if ( ret != PCF2131_OK )
  {
    ret = RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief  Set a timestamp.
 * @param  which_timestamp:= Which timestamp (1-4) to set
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_set_timestamp ( pcf2131_timestamp_t which_timestamp )
{
  int32_t ret = RTC_SUCCESS;

  if ( (which_timestamp < TIMESTAMP_1) || (which_timestamp > TIMESTAMP_4) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  if ( rtc_self->ts_in_use[which_timestamp] )
  {
    return RTC_TIMESTAMP_ALREADY_IN_USE;
  }

  // Active low pins
  HAL_GPIO_WritePin (rtc_self->ts_pins[which_timestamp].port,
                     rtc_self->ts_pins[which_timestamp].pin, GPIO_PIN_RESET);
  tx_thread_sleep (1);
  HAL_GPIO_WritePin (rtc_self->ts_pins[which_timestamp].port,
                     rtc_self->ts_pins[which_timestamp].pin, GPIO_PIN_SET);

  rtc_self->ts_in_use[which_timestamp] = true;

  return ret;
}

/**
 * @brief  Get a timestamp.
 * @param  which_timestamp:= Which timestamp (1-4) to read
 * @param  return_timestamp:= Timestamp as time_t
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_get_timestamp ( pcf2131_timestamp_t which_timestamp,
                                                time_t *return_timestamp )
{
  int32_t ret = RTC_SUCCESS;
  struct tm timestamp_struct;

  if ( (which_timestamp < TIMESTAMP_1) || (which_timestamp > TIMESTAMP_4) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  if ( !rtc_self->ts_in_use[which_timestamp] )
  {
    return RTC_TIMESTAMP_NOT_SET;
  }

  // Grab the timestamp
  ret = pcf2131_get_timestamp (&rtc_self->dev_ctx, which_timestamp, &timestamp_struct);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Clear the timestamp flag
  ret = pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, which_timestamp);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  rtc_self->ts_in_use[which_timestamp] = false;

  *return_timestamp = mktime (&timestamp_struct);

  return ret;
}

/**
 * @brief  Set the alarm
 * @param  alarm_setting:= Settings to be applied to the alarm
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_set_alarm ( rtc_alarm_struct alarm_setting )
{
  int32_t ret = RTC_SUCCESS;

  ret = pcf2131_set_alarm (&rtc_self->dev_ctx, &alarm_setting);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief  Clear a flag in the RTC
 * @param  which_flag:= Flag to be cleared
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_clear_flag ( rtc_flag_t which_flag )
{
  rtc_return_code retval = PCF2131_OK;

  switch ( which_flag )
  {
    case MINUTE_SECOND_FLAG:
      retval |= pcf2131_clear_msf_flag (&rtc_self->dev_ctx);
      break;

    case WATCHDOG_FLAG:
      retval |= pcf2131_clear_watchdog_flag (&rtc_self->dev_ctx);
      break;

    case ALARM_FLAG:
      retval |= pcf2131_clear_alarm_flag (&rtc_self->dev_ctx);
      break;

    case BATTERY_SWITCHOVER_FLAG:
      retval |= pcf2131_clear_battery_switch_over_flag (&rtc_self->dev_ctx);
      break;

    case BATTERY_STATUS_FLAG:
      retval |= pcf2131_clear_battery_status_flag (&rtc_self->dev_ctx);
      break;

    case TIMESTAMP1_FLAG:
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_1);
      break;

    case TIMESTAMP2_FLAG:
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_2);
      break;
      break;

    case TIMESTAMP3_FLAG:
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_3);
      break;

    case TIMESTAMP4_FLAG:
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_4);
      break;

    case ALL_RTC_FLAGS:
      retval |= pcf2131_clear_msf_flag (&rtc_self->dev_ctx);
      retval |= pcf2131_clear_watchdog_flag (&rtc_self->dev_ctx);
      retval |= pcf2131_clear_alarm_flag (&rtc_self->dev_ctx);
      retval |= pcf2131_clear_battery_switch_over_flag (&rtc_self->dev_ctx);
      retval |= pcf2131_clear_battery_status_flag (&rtc_self->dev_ctx);
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_1);
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_2);
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_3);
      retval |= pcf2131_clear_timestamp_flag (&rtc_self->dev_ctx, TIMESTAMP_4);
      break;

    default:
      return RTC_PARAMETERS_INVALID;
  }

  return retval;
}

/**
 * @brief Initialize the SPI bus if not already initialized.
 * @param  None
 * @retval rtc_return_code
 */
static int32_t _ext_rtc_spi_init ( void )
{
  rtc_return_code retval = PCF2131_OK;

  if ( !spi_bus_init_status (rtc_self->rtc_spi_bus->Instance) )
  {
    retval = spi1_init ();
  }

  return retval;
}

/**
 * @brief Deinitialize the SPI bus if it has not already been.
 * @param  None
 * @retval rtc_return_code
 */
static int32_t _ext_rtc_spi_deinit ( void )
{
  int32_t retval = PCF2131_OK;

  if ( spi_bus_init_status (rtc_self->rtc_spi_bus->Instance) )
  {
    retval = spi1_deinit ();
  }

  return retval;
}

/**
 * @brief Blocking SPI read for the Sensor. CS pin handled within function.
 * @param  bus_address - Unused -- used when I2C is the interfacing bus
 * @param  reg_address - register address
 * @param  read_data - read data return pointer
 * @param  data_length - read data length in bytes
 * @retval rtc_return_code
 */
static int32_t _ext_rtc_read_reg_spi ( void *unused_handle, uint16_t unused_bus_address,
                                       uint16_t reg_address, uint8_t *read_data,
                                       uint16_t data_length )
{
  (void) unused_handle;
  (void) unused_bus_address;
  int32_t retval = PCF2131_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE] =
    { 0 };
  uint8_t read_buf[RTC_SPI_BUF_SIZE] =
    { 0 };

  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);

  if ( !((data_length <= sizeof(write_buf)) && ((data_length + 1) <= sizeof(write_buf))) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive_IT (rtc_self->rtc_spi_bus, &(write_buf[0]), &read_buf[0],
                                   data_length + 1)
       != HAL_OK )
  {
    retval = PCF2131_ERROR;
  }

  if ( tx_semaphore_get (rtc_self->spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = PCF2131_ERROR;
  }

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  memcpy (read_data, &read_buf[1], data_length);

  return retval;
}

/**
 * @brief Blocking SPI write for the Sensor. CS pin handled within function.
 * @param  bus_address - Unused -- used when I2C is the interfacing bus
 * @param  reg_address - register address
 * @param  write_data - write data buffer pointer
 * @param  data_length - write data length in bytes
 * @retval rtc_return_code
 */
static int32_t _ext_rtc_write_reg_spi ( void *unused_handle, uint16_t unused_bus_address,
                                        uint16_t reg_address, uint8_t *write_data,
                                        uint16_t data_length )
{
  (void) unused_handle;
  (void) unused_bus_address;
  int32_t retval = PCF2131_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE + 1] =
    { 0 };

  if ( !(data_length <= sizeof(write_buf)) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  write_buf[0] = (uint8_t) reg_address;
  memcpy (&(write_buf[1]), write_data, data_length);

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_Transmit_IT (rtc_self->rtc_spi_bus, write_buf, data_length + 1) != HAL_OK )
  {
    retval = PCF2131_ERROR;
  }

  if ( tx_semaphore_get (rtc_self->spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = PCF2131_ERROR;
  }

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
}

/**
 * @brief Millisecond delay function.
 * @param  delay - delay in milliseconds
 * @retval void
 */
static void _ext_rtc_ms_delay ( uint32_t delay )
{
  if ( delay == 0 )
  {
    tx_thread_relinquish ();
  }
  else
  {
    tx_thread_sleep (delay);
  }
}

static uint8_t __weekday_from_date ( int y, int m, int d )
{
  y += 2000;
  /* wikipedia.org/wiki/Determination_of_the_day_of_the_week#Implementation-dependent_methods */
  uint8_t weekday = (uint8_t) ((d += m < 3 ?
      y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400)
                               % 7);

  return weekday;
}

///**
// * @brief SPI read for the Sensor using DMA. CS pin handled within function.
// * @param  bus_address - Unused -- used when I2C is the interfacing bus
// * @param  reg_address - register address
// * @param  read_data - read data return pointer
// * @param  data_length - read data length in bytes
// * @retval rtc_return_code
// */
//static int32_t _ext_rtc_read_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
//                                           uint16_t reg_address, uint8_t *read_data,
//                                           uint16_t data_length )
//{
//  (void) unused_handle;
//  (void) unused_bus_address;
//  int32_t retval = PCF2131_OK;
//  uint8_t write_buf[RTC_SPI_BUF_SIZE] =
//    { 0 };
//  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);
//
//  assert(data_length <= sizeof(write_buf));
//
//  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);
//
//  if ( HAL_SPI_TransmitReceive_DMA (rtc_self->rtc_spi_bus, write_buf, read_data, data_length)
//       != HAL_OK )
//  {
//    retval = PCF2131_ERROR;
//    goto done;
//  }
//
//  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
//  {
//    retval = PCF2131_ERROR;
//    goto done;
//  }
//
//done:
//
//  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);
//
//  return retval;
//}
//
///**
// * @brief SPI write for the Sensor using DMA. CS pin handled within function.
// * @param  bus_address - Unused -- used when I2C is the interfacing bus
// * @param  reg_address - register address
// * @param  write_data - write data buffer pointer
// * @param  data_length - write data length in bytes
// * @retval rtc_return_code
// */
//static int32_t _ext_rtc_write_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
//                                            uint16_t reg_address, uint8_t *write_data,
//                                            uint16_t data_length )
//{
//  (void) unused_handle;
//  (void) unused_bus_address;
//  int32_t retval = PCF2131_OK;
//  uint8_t write_buf[RTC_SPI_BUF_SIZE + 1] =
//    { 0 };
//
//  assert(data_length <= sizeof(write_buf));
//
//  write_buf[0] = (uint8_t) reg_address;
//  memcpy (&(write_buf[1]), write_data, data_length);
//
//  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);
//
//  if ( HAL_SPI_Transmit_DMA (rtc_self->rtc_spi_bus, write_buf, data_length + 1) != HAL_OK )
//  {
//    retval = PCF2131_ERROR;
//    goto done;
//  }
//
//  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
//  {
//    retval = PCF2131_ERROR;
//  }
//
//done:
//
//  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);
//
//  return retval;
//}

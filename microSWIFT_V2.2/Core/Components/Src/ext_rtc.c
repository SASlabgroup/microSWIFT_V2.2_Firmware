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

static Ext_RTC *self;

/* Core struct functions */
static rtc_return_code _ext_rtc_setup_rtc ( void );
static rtc_return_code _ext_rtc_config_watchdog ( uint32_t period_ms );
static rtc_return_code _ext_rtc_refresh_watchdog ( void );
static rtc_return_code _ext_rtc_set_date_time ( struct tm input_date_time );
static rtc_return_code _ext_rtc_get_date_time ( struct tm *return_date_time );
static rtc_return_code _ext_rtc_set_timestamp ( pcf2131_timestamp_t which_timestamp );
static rtc_return_code _ext_rtc_get_timestamp ( pcf2131_timestamp_t which_timestamp,
                                                time_t *return_timestamp );
static rtc_return_code _ext_rtc_set_alarm ( rtc_alarm_struct alarm_setting );

/* SPI driver functions */
static int32_t _ext_rtc_spi_init ( void );
static int32_t _ext_rtc_spi_deinit ( void );
static int32_t _ext_rtc_read_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
                                                uint16_t reg_address, uint8_t *read_data,
                                                uint16_t data_length );
static int32_t _ext_rtc_write_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
                                                 uint16_t reg_address, uint8_t *write_data,
                                                 uint16_t data_length );
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
                               TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags )
{
  int32_t ret;
  uint8_t register_read = 0;
  // Grab the global struct pointer
  self = struct_ptr;

  self->request_queue = request_queue;
  self->complete_flags = complete_flags;
  self->rtc_spi_bus = rtc_spi_bus;

  self->int_a_pin.port = RTC_INT_A_GPIO_Port;
  self->int_a_pin.pin = RTC_INT_A_Pin;
  self->int_b_pin.port = RTC_INT_B_GPIO_Port;
  self->int_b_pin.pin = RTC_INT_B_Pin;

  self->ts_pins[0].port = RTC_TIMESTAMP_1_GPIO_Port;
  self->ts_pins[0].pin = RTC_TIMESTAMP_1_Pin;

  self->ts_pins[1].port = RTC_TIMESTAMP_2_GPIO_Port;
  self->ts_pins[1].pin = RTC_TIMESTAMP_2_Pin;

  self->ts_pins[2].port = RTC_TIMESTAMP_3_GPIO_Port;
  self->ts_pins[2].pin = RTC_TIMESTAMP_3_Pin;

  self->ts_pins[3].port = RTC_TIMESTAMP_4_GPIO_Port;
  self->ts_pins[3].pin = RTC_TIMESTAMP_4_Pin;

  self->ts_in_use[0] = false;
  self->ts_in_use[1] = false;
  self->ts_in_use[2] = false;
  self->ts_in_use[3] = false;

  self->watchdog_refresh_time_val = 0;

  self->setup_rtc = _ext_rtc_setup_rtc;
  self->config_watchdog = _ext_rtc_config_watchdog;
  self->refresh_watchdog = _ext_rtc_refresh_watchdog;
  self->set_date_time = _ext_rtc_set_date_time;
  self->get_date_time = _ext_rtc_get_date_time;
  self->set_timestamp = _ext_rtc_set_timestamp;
  self->get_timestamp = _ext_rtc_get_timestamp;
  self->set_alarm = _ext_rtc_set_alarm;

  // Register dev_ctx functions
  if ( pcf2131_register_io_functions (&self->dev_ctx, _ext_rtc_spi_init, _ext_rtc_spi_deinit,
                                      _ext_rtc_write_reg_spi_blocking,
                                      _ext_rtc_read_reg_spi_blocking, NULL)
       != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Software reset the RTC
  if ( pcf2131_software_reset (&self->dev_ctx) != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // read the Software Reset register to see if the bit pattern matches default
  ret = self->dev_ctx.bus_read (NULL, 0, RESET_REG_ADDR, &register_read, 1);

  if ( ret != PCF2131_OK || register_read != RESET_REG_RESET_VAL )
  {
    return RTC_SPI_ERROR;
  }

  return RTC_SUCCESS;
}

/**
 * @brief  Apply all required RTC settings, with the exception of the watchdog timer.
 * @param  None
 * @retval rtc_return_code
 */
static rtc_return_code _ext_rtc_setup_rtc ( void )
{
  int32_t ret = RTC_SUCCESS;
  /* set up:
   * power management
   * clock output
   * temperature measurement period
   * interrupts
   * OTP refresh?
   */

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
    self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * (1.0f / 64.0f)) + 1));
  }
  else if ( period_ms > (PCF2131_4HZ_CLK_MAX_PERIOD_MS) )
  {
    clock_select = HZ_1_4;
    self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * (1.0f / 4.0f)) + 1));
  }
  else if ( period_ms > PCF2131_64HZ_CLK_MAX_PERIOD_MS )
  {
    clock_select = HZ_4;
    self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * 4.0f) + 1));
  }
  else
  {
    clock_select = HZ_64;
    self->watchdog_refresh_time_val = (uint8_t) (MILLISECONDS_TO_SECONDS(
        round (((float) period_ms) * 64.0f) + 1));
  }

  ret = pcf2131_watchdog_config_time_source (&self->dev_ctx, clock_select);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  ret = pcf2131_set_watchdog_timer_value (&self->dev_ctx, self->watchdog_refresh_time_val);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  ret = pcf2131_watchdog_irq_config (&self->dev_ctx, true);
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

  ret = pcf2131_set_watchdog_timer_value (&self->dev_ctx, self->watchdog_refresh_time_val);
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
static rtc_return_code _ext_rtc_set_date_time ( struct tm input_date_time )
{
  int32_t ret = RTC_SUCCESS;

  // Convert to BCD format
  struct_tm_dec_to_bcd (&input_date_time);

  if ( (input_date_time.tm_sec == BCD_ERROR) || (input_date_time.tm_min == BCD_ERROR)
       || (input_date_time.tm_hour == BCD_ERROR) || (input_date_time.tm_mday == BCD_ERROR)
       || (input_date_time.tm_mon == BCD_ERROR) || (input_date_time.tm_year == BCD_ERROR)
       || (input_date_time.tm_year == BCD_ERROR) )
  {
    return RTC_PARAMETERS_INVALID;
  }

  ret = pcf2131_set_date_time (&self->dev_ctx, &input_date_time);

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

  ret = pcf2131_get_date_time (&self->dev_ctx, return_date_time);

  if ( ret != PCF2131_OK )
  {
    ret = RTC_SPI_ERROR;
  }

  // Convert to decimal
  struct_tm_bcd_to_dec (return_date_time);

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

  if ( self->ts_in_use[which_timestamp] )
  {
    return RTC_TIMESTAMP_ALREADY_IN_USE;
  }

  // Active low pins
  HAL_GPIO_WritePin (self->ts_pins[which_timestamp].port, self->ts_pins[which_timestamp].pin,
                     GPIO_PIN_RESET);
  tx_thread_sleep (1);
  HAL_GPIO_WritePin (self->ts_pins[which_timestamp].port, self->ts_pins[which_timestamp].pin,
                     GPIO_PIN_SET);

  self->ts_in_use[which_timestamp] = true;

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

  if ( !self->ts_in_use[which_timestamp] )
  {
    return RTC_TIMESTAMP_NOT_SET;
  }

  // Grab the timestamp
  ret = pcf2131_get_timestamp (&self->dev_ctx, which_timestamp, &timestamp_struct);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  // Clear the timestamp flag
  ret = pcf2131_clear_timestamp_flag (&self->dev_ctx, which_timestamp);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  self->ts_in_use[which_timestamp] = false;

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

  ret = pcf2131_set_alarm (&self->dev_ctx, &alarm_setting);
  if ( ret != PCF2131_OK )
  {
    return RTC_SPI_ERROR;
  }

  return ret;
}

/**
 * @brief Initialize the SPI bus if not already initialized.
 * @param  None
 * @retval rtc_return_code
 */
static int32_t _ext_rtc_spi_init ( void )
{
  rtc_return_code retval = PCF2131_OK;

  if ( !spi_bus_init_status (self->rtc_spi_bus->Instance) )
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

  if ( spi_bus_init_status (self->rtc_spi_bus->Instance) )
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
static int32_t _ext_rtc_read_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
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

  assert((data_length <= sizeof(write_buf)) && ((data_length + 1) <= sizeof(write_buf)));

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive (self->rtc_spi_bus, &(write_buf[0]), &read_buf[0], data_length + 1,
  RTC_SPI_TIMEOUT)
       != HAL_OK )
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
static int32_t _ext_rtc_write_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
                                                 uint16_t reg_address, uint8_t *write_data,
                                                 uint16_t data_length )
{
  (void) unused_handle;
  (void) unused_bus_address;
  int32_t retval = PCF2131_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE + 1] =
    { 0 };

  assert(data_length <= sizeof(write_buf));

  write_buf[0] = (uint8_t) reg_address;
  memcpy (&(write_buf[1]), write_data, data_length);

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_Transmit (self->rtc_spi_bus, write_buf, data_length + 1, RTC_SPI_TIMEOUT) != HAL_OK )
  {
    retval = PCF2131_ERROR;
  }

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
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
//  if ( HAL_SPI_TransmitReceive_DMA (self->rtc_spi_bus, write_buf, read_data, data_length)
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
//  if ( HAL_SPI_Transmit_DMA (self->rtc_spi_bus, write_buf, data_length + 1) != HAL_OK )
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

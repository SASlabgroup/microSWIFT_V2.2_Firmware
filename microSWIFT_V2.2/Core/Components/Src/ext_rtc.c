/*
 * ext_rtc.c
 *
 *  Created on: Jul 29, 2024
 *      Author: philbush
 */

#include "ext_rtc.h"
#include "pcf2131_reg.h"
#include "spi.h"
#include "gpio.h"
#include "app_threadx.h"
#include "main.h"

static Ext_RTC *self;

/* Core struct functions */
static ext_rtc_return_code _ext_rtc_config_watchdog ( uint32_t period_ms );
static ext_rtc_return_code _ext_rtc_refresh_watchdog ( void );
static ext_rtc_return_code _ext_rtc_set_date_time ( struct tm input_date_time );
static ext_rtc_return_code _ext_rtc_get_date_time ( struct tm *return_date_time );
static ext_rtc_return_code _ext_rtc_set_timestamp ( rtc_timestamp_t which_timestamp );
static ext_rtc_return_code _ext_rtc_get_timestamp ( rtc_timestamp_t which_timestamp,
                                                    time_t *return_timestamp );
static ext_rtc_return_code _ext_rtc_set_alarm ( rtc_alarm_struct alarm_setting );

/* SPI driver functions */
static int32_t _ext_rtc_spi_init ( void );
static int32_t _ext_rtc_spi_deinit ( void );
static int32_t _ext_rtc_read_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
                                                uint16_t reg_address, uint8_t *read_data,
                                                uint16_t data_length );
static int32_t _ext_rtc_write_reg_spi_blocking ( void *unused_handle, uint16_t unused_bus_address,
                                                 uint16_t reg_address, uint8_t *write_data,
                                                 uint16_t data_length );
static int32_t _ext_rtc_read_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
                                           uint16_t reg_address, uint8_t *read_data,
                                           uint16_t data_length );
static int32_t _ext_rtc_write_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
                                            uint16_t reg_address, uint8_t *write_data,
                                            uint16_t data_length );

/**
 * @brief  Initialize the Ext_RTC struct
 *
 * @note   INT_A pin will be used for the watchdog, INT_B pin will be used for the Alarm function.
 *
 * @param  struct_ptr:= Global struct pointer, saved locally as static pointer
 * @param  rtc_spi_bus:= Handle for SPI bus
 * @param  messaging_queue:= Pointer to global messaging queue for inbound requests
 * @retval ext_rtc_return_code
 */
ext_rtc_return_code ext_rtc_init ( Ext_RTC *struct_ptr, SPI_HandleTypeDef *rtc_spi_bus,
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
  self->ts_1_pin.port = RTC_TIMESTAMP_1_GPIO_Port;
  self->ts_1_pin.pin = RTC_TIMESTAMP_1_Pin;
  self->ts_2_pin.port = RTC_TIMESTAMP_2_GPIO_Port;
  self->ts_2_pin.pin = RTC_TIMESTAMP_2_Pin;
  self->ts_3_pin.port = RTC_TIMESTAMP_3_GPIO_Port;
  self->ts_3_pin.pin = RTC_TIMESTAMP_3_Pin;
  self->ts_4_pin.port = RTC_TIMESTAMP_4_GPIO_Port;
  self->ts_4_pin.pin = RTC_TIMESTAMP_4_Pin;

  self->ts1_in_use = false;
  self->ts2_in_use = false;
  self->ts3_in_use = false;
  self->ts4_in_use = false;

  self->config_watchdog = _ext_rtc_config_watchdog;
  self->refresh_watchdog = _ext_rtc_refresh_watchdog;
  self->set_date_time = _ext_rtc_set_date_time;
  self->get_date_time = _ext_rtc_get_date_time;
  self->set_timestamp = _ext_rtc_set_timestamp;
  self->get_timestamp = _ext_rtc_get_timestamp;
  self->set_alarm = _ext_rtc_set_alarm;

  // Register dev_ctx functions
  if ( pcf2131_register_io_functions (&self->dev_ctx, _ext_rtc_spi_init, _ext_rtc_spi_deinit,
                                      _ext_rtc_write_reg_spi_dma, _ext_rtc_read_reg_spi_dma, NULL)
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

#error "set up power management and interrupts here"

  return RTC_SUCCESS;
}

/**
 * @brief  Configure the watchdog, hook it up to INT pin, set the timer.
 *         !!! The watchdog will start after this function returns success !!!
 * @param  period_ms:= Watchdog refresh interval in milliseconds
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_config_watchdog ( uint32_t period_ms )
{
  int32_t ret = RTC_SUCCESS;
  watchdog_time_source_t clock_select;
  uint8_t timer_period;

  // We'll establish a minimum refresh interval of 10 seconds
  if ( period_ms < RTC_WATCHDOG_MIN_REFRESH )
  {
    return EXT_RTC_PARAMETERS_INVALID;
  }

  // Figure out what watchdog clock rate to use
  // Reference Table 59 in datasheet
  if ( period_ms > SECONDS_TO_MILLISECONDS(1020) )
  {

    if ( period_ms > SECONDS_TO_MILLISECONDS(16320) )
    {
      // Unacheivably long interval
      return EXT_RTC_PARAMETERS_INVALID;
    }

    clock_select = HZ_1_64;
    timer_period = (uint8_t) (MILLISECONDS_TO_SECONDS(
        (uint32) round (((float) period_ms) * (1.0f / 64.0f)) + 1));
  }
  else if ( period_ms > (63744) )
  {
    clock_select = HZ_1_4;
    timer_period = (uint8_t) (MILLISECONDS_TO_SECONDS(
        (uint32) round (((float) period_ms) * (1.0f / 4.0f)) + 1));
  }
  else if ( period_ms > 3984 )
  {
    clock_select = HZ_4;
    timer_period = (uint8_t) (MILLISECONDS_TO_SECONDS(
        (uint32) round (((float) period_ms) * 4.0f) + 1));
  }
  else
  {
    clock_select = HZ_64;
    timer_period = (uint8_t) (MILLISECONDS_TO_SECONDS(
        (uint32) round (((float) period_ms) * 64.0f) + 1));
  }

  ret = pcf2131_watchdog_config_time_source (&self->dev_ctx, clock_select);
  if ( ret != PCF2131_OK )
  {
    return EXT_RTC_SPI_ERROR;
  }

  ret = pcf2131_set_watchdog_timer_value (&self->dev_ctx, timer_period);
  if ( ret != PCF2131_OK )
  {
    return EXT_RTC_SPI_ERROR;
  }

  ret = pcf2131_watchdog_irq_signal_config (&self->dev_ctx, true);
  if ( ret != PCF2131_OK )
  {
    return EXT_RTC_SPI_ERROR;
  }

  return return_code;
}

/**
 * @brief  Refresh the watchdog.
 * @param  None
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_refresh_watchdog ( void )
{
  ext_rtc_return_code return_code = RTC_SUCCESS;

  return return_code;
}

/**
 * @brief  Set the RTC date and Timer registers
 * @param  input_date_time:= struct tm containing the desired time settings
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_set_date_time ( struct tm input_date_time )
{
  ext_rtc_return_code return_code = RTC_SUCCESS;

  return return_code;
}

/**
 * @brief  Get the current date/time from the RTC
 * @param  return_date_time:= Return pointer for struct tm
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_get_date_time ( struct tm *return_date_time )
{
  ext_rtc_return_code return_code = RTC_SUCCESS;

  return return_code;
}

/**
 * @brief  Set a timestamp.
 * @param  which_timestamp:= Which timestamp (1-4) to set
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_set_timestamp ( rtc_timestamp_t which_timestamp )
{

}

/**
 * @brief  Get a timestamp.
 * @param  which_timestamp:= Which timestamp (1-4) to read
 * @param  return_timestamp:= Timestamp as time_t
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_get_timestamp ( rtc_timestamp_t which_timestamp,
                                                    time_t *return_timestamp )
{
  ext_rtc_return_code return_code = RTC_SUCCESS;

  return return_code;
}

/**
 * @brief  Set the alarm
 * @param  alarm_setting:= Settings to be applied to the alarm
 * @retval ext_rtc_return_code
 */
static ext_rtc_return_code _ext_rtc_set_alarm ( rtc_alarm_struct alarm_setting )
{
  ext_rtc_return_code return_code = RTC_SUCCESS;

  return return_code;
}

/**
 * @brief Initialize the SPI bus if not already initialized.
 * @param  None
 * @retval ext_rtc_return_code
 */
static int32_t _ext_rtc_spi_init ( void )
{
  ext_rtc_return_code retval = PCF2131_OK;

  if ( !spi_bus_init_status (self->rtc_spi_bus->Instance) )
  {
    retval = spi1_init ();

  }

  return retval;
}

/**
 * @brief Deinitialize the SPI bus if it has not already been.
 * @param  None
 * @retval ext_rtc_return_code
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
 * @retval ext_rtc_return_code
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
  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);

  assert(data_length <= sizeof(write_buf));

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive (self->rtc_spi_bus, &(write_buf[0]), read_data, data_length,
  RTC_SPI_TIMEOUT)
       != HAL_OK )
  {
    retval = PCF2131_ERROR;
  }

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
}

/**
 * @brief Blocking SPI write for the Sensor. CS pin handled within function.
 * @param  bus_address - Unused -- used when I2C is the interfacing bus
 * @param  reg_address - register address
 * @param  write_data - write data buffer pointer
 * @param  data_length - write data length in bytes
 * @retval ext_rtc_return_code
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

/**
 * @brief SPI read for the Sensor using DMA. CS pin handled within function.
 * @param  bus_address - Unused -- used when I2C is the interfacing bus
 * @param  reg_address - register address
 * @param  read_data - read data return pointer
 * @param  data_length - read data length in bytes
 * @retval ext_rtc_return_code
 */
static int32_t _ext_rtc_read_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
                                           uint16_t reg_address, uint8_t *read_data,
                                           uint16_t data_length )
{
  (void) unused_handle;
  (void) unused_bus_address;
  int32_t retval = PCF2131_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE] =
    { 0 };
  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);

  assert(data_length <= sizeof(write_buf));

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive_DMA (self->rtc_spi_bus, write_buf, read_data, data_length)
       != HAL_OK )
  {
    retval = PCF2131_ERROR;
    goto done;
  }

  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = PCF2131_ERROR;
    goto done;
  }

done:

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
}

/**
 * @brief SPI write for the Sensor using DMA. CS pin handled within function.
 * @param  bus_address - Unused -- used when I2C is the interfacing bus
 * @param  reg_address - register address
 * @param  write_data - write data buffer pointer
 * @param  data_length - write data length in bytes
 * @retval ext_rtc_return_code
 */
static int32_t _ext_rtc_write_reg_spi_dma ( void *unused_handle, uint16_t unused_bus_address,
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

  if ( HAL_SPI_Transmit_DMA (self->rtc_spi_bus, write_buf, data_length + 1) != HAL_OK )
  {
    retval = PCF2131_ERROR;
    goto done;
  }

  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = PCF2131_ERROR;
  }

done:

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
}

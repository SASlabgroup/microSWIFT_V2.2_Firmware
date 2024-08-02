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

static SPI_HandleTypeDef *rtc_spi_bus = &hspi1;

static int32_t ext_rtc_spi_init ( void );
static int32_t ext_rtc_spi_deinit ( void );
static int32_t ext_rtc_read_reg_spi_blocking ( uint16_t bus_address, uint16_t reg_address,
                                               uint8_t *read_data, uint16_t data_length );
static int32_t ext_rtc_write_reg_spi_blocking ( uint16_t bus_address, uint16_t reg_address,
                                                uint8_t *write_data, uint16_t data_length );
static int32_t ext_rtc_read_reg_spi_dma ( uint16_t bus_address, uint16_t reg_address,
                                          uint8_t *read_data, uint16_t data_length );
static int32_t ext_rtc_write_reg_spi_dma ( uint16_t bus_address, uint16_t reg_address,
                                           uint8_t *write_data, uint16_t data_length );
static int32_t ext_rtc_get_access_lock ( void );
static int32_t ext_rtc_release_access_lock ( void );

/**
 * @brief Initialize the SPI bus if not already initialized.
 * @param  None
 * @retval ext_rtc_return_code
 */
static int32_t ext_rtc_spi_init ( void )
{
  ext_rtc_return_code retval = RTC_OK;

  if ( !spi1_bus_init_status () )
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
static int32_t ext_rtc_spi_deinit ( void )
{
  int32_t retval = RTC_OK;

  if ( spi1_bus_init_status () )
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
static int32_t ext_rtc_read_reg_spi_blocking ( uint16_t bus_address, uint16_t reg_address,
                                               uint8_t *read_data, uint16_t data_length )
{
  (void) bus_address;
  int32_t retval = RTC_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE] =
    { 0 };
  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);

  assert(data_length <= sizeof(write_buffer));

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive (rtc_spi_bus, &reg, read_data, data_length, RTC_SPI_TIMEOUT)
       != HAL_OK )
  {
    retval = RTC_SPI_ERROR;
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
static int32_t ext_rtc_write_reg_spi_blocking ( uint16_t bus_address, uint16_t reg_address,
                                                uint8_t *write_data, uint16_t data_length )
{
  (void) bus_address;
  int32_t retval = RTC_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE + 1] =
    { 0 };

  assert(data_length <= sizeof(write_buffer));

  write_buffer[0] = (uint8_t) reg_address;
  memcpy (&(buf[1]), write_data, data_length);

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_Transmit (rtc_spi_bus, buf, data_length + 1, RTC_SPI_TIMEOUT) != HAL_OK )
  {
    retval = RTC_SPI_ERROR;
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
static int32_t ext_rtc_read_reg_spi_dma ( uint16_t bus_address, uint16_t reg_address,
                                          uint8_t *read_data, uint16_t data_length )
{
  (void) bus_address;
  int32_t retval = RTC_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE] =
    { 0 };
  write_buf[0] = (uint8_t) (reg_address | PCF2131_SPI_READ_BIT);

  assert(data_length <= sizeof(write_buffer));

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_TransmitReceive_DMA (rtc_spi_bus, &reg, read_data, data_length) != HAL_OK )
  {
    retval = RTC_SPI_ERROR;
    goto done;
  }

  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = RTC_SPI_ERROR;
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
static int32_t ext_rtc_write_reg_spi_dma ( uint16_t bus_address, uint16_t reg_address,
                                           uint8_t *write_data, uint16_t data_length )
{
  (void) bus_address;
  int32_t retval = RTC_OK;
  uint8_t write_buf[RTC_SPI_BUF_SIZE + 1] =
    { 0 };

  assert(data_length <= sizeof(write_buffer));

  write_buffer[0] = (uint8_t) reg_address;
  memcpy (&(buf[1]), write_data, data_length);

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_RESET);

  if ( HAL_SPI_Transmit_DMA (rtc_spi_bus, buf, data_length + 1) != HAL_OK )
  {
    retval = RTC_SPI_ERROR;
    goto done;
  }

  if ( tx_semaphore_get (&ext_rtc_spi_sema, RTC_SPI_TIMEOUT) != TX_SUCCESS )
  {
    retval = RTC_SPI_ERROR;
  }

done:

  HAL_GPIO_WritePin (RTC_SPI_CS_GPIO_Port, RTC_SPI_CS_Pin, GPIO_PIN_SET);

  return retval;
}

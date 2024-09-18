/*
 * shared_i2c_driver.c
 *
 *  Created on: Sep 17, 2024
 *      Author: philbush
 */

#include "shared_i2c_driver.h"

static i2c_init_fn shadow_init_fn;
static i2c_deinit_fn shadow_deinit_fn;
static I2C_HandleTypeDef *shadow_i2c_handle;

static int32_t shared_i2c_init ( void );
static int32_t shared_i2c_deinit ( void );
static int32_t shared_i2c_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size,
                                 uint32_t timeout_ticks );
static int32_t shared_i2c_write ( void *driver_ptr, const uint8_t *write_buf, uint16_t size,
                                  uint32_t timeout_ticks );

void shared_i2c_register_io_functions ( shared_i2c_driver *driver_ptr,
                                        I2C_HandleTypeDef *i2c_handle, TX_MUTEX *i2c_mutex,
                                        i2c_init_fn init, i2c_deinit_fn deinit,
                                        i2c_read_fn override_read_fn,
                                        i2c_write_fn override_write_fn )
{
  shadow_i2c_handle = i2c_handle;
  driver_ptr->i2c_handle = i2c_handle;
  driver_ptr->i2c_mutex = i2c_mutex;
  shadow_init_fn = init;
  driver_ptr->init = shared_i2c_init;
  shadow_deinit_fn = deinit;
  driver_ptr->deinit = shared_i2c_deinit;
  driver_ptr->deinit = deinit;

  if ( override_read_fn != NULL )
  {
    driver_ptr->read = override_read_fn;
  }
  else
  {
    driver_ptr->read = shared_i2c_read;
  }

  if ( override_write_fn != NULL )
  {
    driver_ptr->write = override_write_fn;
  }
  else
  {
    driver_ptr->write = shared_i2c_write;
  }
}

void shared_i2c_set_timeout_ticks ( shared_i2c_driver *driver_ptr, ULONG tx_timeout_ticks,
                                    ULONG rx_timeout_ticks )
{
  driver_ptr->tx_timeout_ticks = tx_timeout_ticks;
  driver_ptr->rx_timeout_ticks = rx_timeout_ticks;
}

static int32_t shared_i2c_init ( void )
{
  // Check if it is initialized
  if ( !(shadow_i2c_handle->Instance->CR1 & I2C_CR1_PE) )
  {
    return shadow_init_fn ();
  }
}

static int32_t shared_i2c_deinit ( void )
{
  // Check if it is deinitialized
  if ( shadow_i2c_handle->Instance->CR1 & I2C_CR1_PE )
  {
    return shadow_deinit_fn ();
  }
}

static int32_t shared_i2c_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size,
                                 uint32_t timeout_ticks )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Receive_DMA (driver_handle->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERR;
  }

  return UART_OK;
}

static int32_t shared_i2c_write ( void *driver_ptr, const uint8_t *write_buf, uint16_t size,
                                  uint32_t timeout_ticks )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Transmit_DMA (driver_handle->uart_handle, write_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERR;
  }

  return UART_OK;
}

/*
 * generic_uart_driver.c
 *
 *  Created on: Aug 5, 2024
 *      Author: philbush
 */

#include "generic_uart_driver.h"
#include "stddef.h"

static int32_t _generic_uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size );
static int32_t _generic_uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size );

void generic_uart_register_io_functions ( generic_uart_driver *driver_ptr,
                                          UART_HandleTypeDef *uart_handle, TX_SEMAPHORE *uart_sema,
                                          uart_init_fn init, uart_deinit_fn deinit,
                                          uart_read_fn override_read_fn,
                                          uart_write_fn override_write_fn )
{
  driver_ptr->uart_handle = uart_handle;
  driver_ptr->uart_sema = uart_sema;
  driver_ptr->init = init;
  driver_ptr->deinit = deinit;

  if ( override_read_fn != NULL )
  {
    driver_ptr->read = override_read_fn;
  }
  else
  {
    driver_ptr->read = _generic_uart_read;
  }

  if ( override_write_fn != NULL )
  {
    driver_ptr->write = override_write_fn;
  }
  else
  {
    driver_ptr->write = _generic_uart_write;
  }
}

void generic_uart_set_timeout_ticks ( generic_uart_driver *driver_ptr, ULONG tx_timeout_ticks,
                                      ULONG rx_timeout_ticks )
{
  driver_ptr->tx_timeout_ticks = tx_timeout_ticks;
  driver_ptr->rx_timeout_ticks = rx_timeout_ticks;
}

static int32_t _generic_uart_read ( void *driver_ptr, uint8_t *read_buf, uint16_t size )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Receive_DMA (driver_handle->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, driver_handle->rx_timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERR;
  }

  return UART_OK;
}

static int32_t _generic_uart_write ( void *driver_ptr, uint8_t *write_buf, uint16_t size )
{
  generic_uart_driver *driver_handle = (generic_uart_driver*) driver_ptr;

  HAL_UART_Transmit_DMA (driver_handle->uart_handle, write_buf, size);

  if ( tx_semaphore_get (driver_handle->uart_sema, driver_handle->tx_timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERR;
  }

  return UART_OK;
}

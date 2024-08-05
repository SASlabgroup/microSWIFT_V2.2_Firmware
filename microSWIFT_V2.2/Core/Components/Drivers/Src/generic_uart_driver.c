/*
 * generic_uart_driver.c
 *
 *  Created on: Aug 5, 2024
 *      Author: philbush
 */

#include "generic_uart_driver.h"

int32_t generic_uart_init ( generic_uart_driver *driver_ptr, UART_HandleTypeDef *uart_handle,
                            TX_SEMAPHORE *uart_sema, init_fn init )
{
  driver_ptr->uart_handle = uart_handle;
  driver_ptr->uart_sema = uart_sema;

  return init ();
}

int32_t generic_uart_deinit ( generic_uart_driver *driver_ptr )
{
  if ( HAL_UART_DeInit (driver_ptr->uart_handle) != HAL_OK )
  {
    return UART_ERROR;
  }

  return UART_OK;
}

int32_t generic_uart_read ( generic_uart_driver *driver_ptr, uint8_t *read_buf, uint16_t size,
                            uint32_t timeout_ticks )
{
  HAL_UART_Receive_DMA (driver_ptr->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_ptr->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERROR;
  }

  return UART_OK;
}

int32_t generic_uart_write ( generic_uart_driver *driver_ptr, const uint8_t *write_buf,
                             uint16_t size, uint32_t timeout_ticks )
{
  HAL_UART_Transmit_DMA (driver_ptr->uart_handle, read_buf, size);

  if ( tx_semaphore_get (driver_ptr->uart_sema, timeout_ticks) != TX_SUCCESS )
  {
    return UART_ERROR;
  }

  return UART_OK;
}

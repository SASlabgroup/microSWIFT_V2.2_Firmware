/*
 * generic_uart_driver.h
 *
 *  Created on: Aug 5, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_DRIVERS_INC_GENERIC_UART_DRIVER_H_
#define COMPONENTS_DRIVERS_INC_GENERIC_UART_DRIVER_H_

#include "tx_api.h"
#include "usart.h"

#define GENERIC_UART_OK 0
#define GENERIC_UART_ERROR -1

typedef int32_t (*init_fn) ( void );
typedef int32_t (*deinit_fn) ( void );

typedef struct
{
  UART_HandleTypeDef *uart_handle;
  TX_SEMAPHORE *uart_sema;

  init_fn init;
  deinit_fn deinit;
} generic_uart_driver;

int32_t generic_uart_init ( generic_uart_driver *driver_ptr, UART_HandleTypeDef *uart_handle,
                            TX_SEMAPHORE *uart_sema, init_fn init, deinit_fn deinit );
int32_t generic_uart_deinit ( void );
int32_t generic_uart_read ( generic_uart_driver *driver_ptr, uint8_t *read_buf, uint16_t size,
                            uint32_t timeout_ticks );
int32_t generic_uart_write ( generic_uart_driver *driver_ptr, const uint8_t *write_buf,
                             uint16_t size, uint32_t timeout_ticks );

#endif /* COMPONENTS_DRIVERS_INC_GENERIC_UART_DRIVER_H_ */

/*
 * CT.h
 *      Author: Laura
 */

#ifndef SRC_ACCELEROMETER_H_
#define SRC_ACCELEROMETER_H_

#include "generic_uart_driver.h"
#include "microSWIFT_return_codes.h"
#include "tx_api.h"

#define ACCEL_MAX_UART_TX_TICKS (TX_TIMER_TICKS_PER_SECOND)

struct Accelerometer;

typedef struct Accelerometer {
  // Our global configuration struct -- not actually used by the ct?
  // But the accelerometer will probably (eventually) need it for
  // setting thresholds for wake-on-shake.
  // microSWIFT_configuration  *global_config;
  generic_uart_driver uart_driver;

  // CT had ct_uart_handle, which I think is unused?
  // CT had {rx,tx}_dma_handle, but they're ONLY used in the deinit function
  // TODO(lindzey): Figure out if these are actually used, and if so, whether
  //    they should be grouped in the uart_driver rather than the Accelerometer
  //    struct
  DMA_HandleTypeDef *tx_dma_handle;
  DMA_HandleTypeDef *rx_dma_handle;

  uSWIFT_return_code_t (*self_test)(struct Accelerometer *accel);
  uSWIFT_return_code_t (*uart_init)(struct Accelerometer *accel);
  uSWIFT_return_code_t (*uart_deinit)(struct Accelerometer *accel);
  uSWIFT_return_code_t (*uart_reset)(struct Accelerometer *accel);

} Accelerometer;

void accelerometer_init(Accelerometer *struct_ptr,
                        UART_HandleTypeDef *accel_uart_handle,
                        DMA_HandleTypeDef *tx_dma_handle,
                        DMA_HandleTypeDef *rx_dma_handle,
                        TX_SEMAPHORE *uart_sema);

#endif // 1SRC_ACCELEROMETER_H_
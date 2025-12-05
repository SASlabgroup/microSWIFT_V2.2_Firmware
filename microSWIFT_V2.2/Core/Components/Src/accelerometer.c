/*
 * Acceperometer.c
 *
 *      Author: Laura
 */

#include "accelerometer.h"

// Private function prototypes; since not part of public interface, did not
// include them in .h file
uSWIFT_return_code_t _accel_self_test(Accelerometer *accel);
uSWIFT_return_code_t _accel_uart_init(Accelerometer *accel);
uSWIFT_return_code_t _accel_uart_deinit(Accelerometer *accel);
uSWIFT_return_code_t _accel_uart_reset(Accelerometer *accel);

// NB: I'm trying to avoid creating an accel_self, but I guess we might
// wind up needing one if any of these functions need to be void-void.

void accelerometer_init(Accelerometer *accel, UART_HandleTypeDef *uart_handle,
                        DMA_HandleTypeDef *tx_dma_handle,
                        DMA_HandleTypeDef *rx_dma_handle,
                        TX_SEMAPHORE *uart_sema) {

  accel->self_test = _accel_self_test;
  accel->uart_init = _accel_uart_init;
  accel->uart_deinit = _accel_uart_deinit;
  accel->uart_reset = _accel_uart_reset;

  // the read/write functions
  accel->tx_dma_handle = tx_dma_handle;
  accel->rx_dma_handle = rx_dma_handle;
  // args are: driver_ptr, uart_handle, uart_sema, uart_init, uart_deinit,
  // override_read_fn, override_write_fn
  // We do not (for now?) need to overwrite read/write.
  generic_uart_register_io_functions(&accel->uart_driver, uart_handle,
                                     uart_sema, usart2_init, usart2_deinit,
                                     NULL, NULL);
}

uSWIFT_return_code_t _accel_self_test(Accelerometer *accel) {
  int32_t ret;
  char command[] = {0xAC, 0x02, 0x00, 0x00, 0x00};
  static int command_len = 5;
  ret = accel->uart_driver.write(&accel->uart_driver, (uint8_t *)&(command[0]),
                                 command_len, ACCEL_MAX_UART_TX_TICKS);

  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  }
  return uSWIFT_SUCCESS;
}

uSWIFT_return_code_t _accel_uart_init(Accelerometer *accel) {
  int32_t ret;
  ret = accel->uart_driver.init();
  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  } else {
    return uSWIFT_SUCCESS;
  }
}

uSWIFT_return_code_t _accel_uart_deinit(Accelerometer *accel) {
  accel->uart_driver.deinit();

  HAL_DMA_DeInit(accel->tx_dma_handle);
  HAL_DMA_DeInit(accel->rx_dma_handle);

  // (These are linked to the Expansion port in app_threadx.c)
  HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel3_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

  HAL_NVIC_DisableIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_DisableIRQ(GPDMA1_Channel3_IRQn);
  HAL_NVIC_DisableIRQ(USART2_IRQn);

  return uSWIFT_SUCCESS;
}

uSWIFT_return_code_t _accel_uart_reset(Accelerometer *accel) {
  // TODO: Fil this in if I use it!
  return uSWIFT_SUCCESS;
}

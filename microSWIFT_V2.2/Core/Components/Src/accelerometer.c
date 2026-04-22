/*
 * Acceperometer.c
 *
 *      Author: Laura
 */

#include "accelerometer.h"

Accelerometer *accel_self;

// Private function prototypes; since not part of public interface, did not
// include them in .h file
uSWIFT_return_code_t _accel_self_test(accel_self_test_result_t *result);
uSWIFT_return_code_t _accel_start_sampling(void);
uSWIFT_return_code_t _accel_uart_init(void);
uSWIFT_return_code_t _accel_uart_deinit(void);
uSWIFT_return_code_t _accel_uart_reset(void);
static void _accel_power_on(void);
static void _accel_power_off(void);

// NB: I'm trying to avoid creating an accel_self, but I guess we might
// wind up needing one if any of these functions need to be void-void.

void accelerometer_init(Accelerometer *accel, UART_HandleTypeDef *uart_handle,
                        DMA_HandleTypeDef *tx_dma_handle,
                        DMA_HandleTypeDef *rx_dma_handle,
                        TX_SEMAPHORE *uart_sema) {
  accel_self = accel;

  accel_self->self_test = _accel_self_test;
  accel_self->start_sampling = _accel_start_sampling;
  accel_self->uart_init = _accel_uart_init;
  accel_self->uart_deinit = _accel_uart_deinit;
  accel_self->uart_reset = _accel_uart_reset;
  accel_self->power_on = _accel_power_on;
  accel_self->power_off = _accel_power_off;

  // the read/write functions
  accel_self->tx_dma_handle = tx_dma_handle;
  accel_self->rx_dma_handle = rx_dma_handle;
  // Final two args are: override_read_fn, override_write_fn
  // We do not (for now?) need to overwrite read/write.
  generic_uart_register_io_functions(&accel_self->uart_driver, uart_handle,
                                     uart_sema, usart2_init, usart2_deinit,
                                     NULL, NULL);
}

void _accel_power_on(void) {
  HAL_GPIO_WritePin(EXP_GPIO_1_GPIO_Port, EXP_GPIO_1_Pin, GPIO_PIN_SET);
}

void _accel_power_off(void) {
  HAL_GPIO_WritePin(EXP_GPIO_1_GPIO_Port, EXP_GPIO_1_Pin, GPIO_PIN_RESET);
}

uSWIFT_return_code_t _accel_start_sampling(void) {
  const char *start_sampling_command = "RW";
  UINT ret;
  ret = accel_self->uart_driver.write(
      &accel_self->uart_driver, (uint8_t *)&(start_sampling_command[0]),
      strlen(start_sampling_command), ACCEL_MAX_UART_TX_TICKS);
  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  }
}

uSWIFT_return_code_t _accel_self_test(accel_self_test_result_t *result) {
  int32_t ret;
  // char self_test_command[] = {0x53, 0x54};  // ST
  const char *self_test_command = "ST";
  ret = accel_self->uart_driver.write(
      &accel_self->uart_driver, (uint8_t *)&(self_test_command[0]),
      strlen(self_test_command), ACCEL_MAX_UART_TX_TICKS);
  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  }

  // TODO: It'd probably be cleaner to add some sentinel bytes to the start of
  // the struct, rather than having transmit and receive sides doing this.
  static int response_length = 2 + sizeof(accel_self_test_result_t);
  char self_test_response[response_length];
  memset(self_test_response, 0, response_length);
  ret = accel_self->uart_driver.read(&accel_self->uart_driver,
                                     (uint8_t *)&(self_test_response[0]),
                                     response_length, ACCEL_MAX_UART_TX_TICKS);
  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  }
  memcpy(result, &self_test_response[2], sizeof(accel_self_test_result_t));

  return uSWIFT_SUCCESS;
}

uSWIFT_return_code_t _accel_uart_init() {
  int32_t ret;
  ret = accel_self->uart_driver.init();
  if (UART_OK != ret) {
    return uSWIFT_IO_ERROR;
  } else {
    return uSWIFT_SUCCESS;
  }
}

uSWIFT_return_code_t _accel_uart_deinit() {
  accel_self->uart_driver.deinit();

  HAL_DMA_DeInit(accel_self->tx_dma_handle);
  HAL_DMA_DeInit(accel_self->rx_dma_handle);

  // (These are linked to the Expansion port in app_threadx.c)
  HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel3_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

  HAL_NVIC_DisableIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_DisableIRQ(GPDMA1_Channel3_IRQn);
  HAL_NVIC_DisableIRQ(USART2_IRQn);

  return uSWIFT_SUCCESS;
}

uSWIFT_return_code_t _accel_uart_reset() {
  // TODO: Fil this in if I use it!
  return uSWIFT_SUCCESS;
}

/*
 * interrupt_callbacks.c
 *
 *  Created on: Aug 2, 2024
 *      Author: philbush
 */

#include "spi.h"
#include "app_threadx.h"
#include "usart.h"

/**
 * @brief Tx Transfer completed callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_TxCpltCallback ( SPI_HandleTypeDef *hspi )
{
  if ( hspi->Instance == RTC_SPI )
  {
    (void) tx_semaphore_put (&ext_rtc_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_1 )
  {
    (void) tx_semaphore_put (&aux_spi_1_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_2 )
  {
    (void) tx_semaphore_put (&aux_spi_2_spi_sema);
  }
}

/**
 * @brief Rx Transfer completed callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_RxCpltCallback ( SPI_HandleTypeDef *hspi )
{
  if ( hspi->Instance == RTC_SPI )
  {
    (void) tx_semaphore_put (&ext_rtc_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_1 )
  {
    (void) tx_semaphore_put (&aux_spi_1_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_2 )
  {
    (void) tx_semaphore_put (&aux_spi_2_spi_sema);
  }
}

/**
 * @brief Tx and Rx Transfer completed callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback ( SPI_HandleTypeDef *hspi )
{
  if ( hspi->Instance == RTC_SPI )
  {
    (void) tx_semaphore_put (&ext_rtc_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_1 )
  {
    (void) tx_semaphore_put (&aux_spi_1_spi_sema);
  }
  else if ( hspi->Instance == AUX_SPI_2 )
  {
    (void) tx_semaphore_put (&aux_spi_2_spi_sema);
  }
}

/**
 * @brief Tx Transfer completed callback.
 * @param huart UART handle.
 * @retval None
 */
void HAL_UART_TxCpltCallback ( UART_HandleTypeDef *huart )
{
  if ( huart->Instance == CT_UART )
  {
    (void) tx_semaphore_put (&ct_uart_sema);
  }
  else if ( huart->Instance == IRIDIUM_UART )
  {
    (void) tx_semaphore_put (&iridium_uart_sema);
  }
  else if ( huart->Instance == GNSS_UART )
  {
    (void) tx_semaphore_put (&gnss_uart_sema);
  }
  else if ( huart->Instance == AUX_UART_1 )
  {
    (void) tx_semaphore_put (&aux_uart_1_sema);
  }
  else if ( huart->Instance == AUX_UART_2 )
  {
    (void) tx_semaphore_put (&aux_uart_2_sema);
  }
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  huart UART handle.
 * @retval None
 */
void HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart )
{
  if ( huart->Instance == CT_UART )
  {
    (void) tx_semaphore_put (&ct_uart_sema);
  }
  else if ( huart->Instance == IRIDIUM_UART )
  {
    (void) tx_semaphore_put (&iridium_uart_sema);
  }
  else if ( huart->Instance == GNSS_UART )
  {
    (void) tx_semaphore_put (&gnss_uart_sema);
  }
  else if ( huart->Instance == AUX_UART_1 )
  {
    (void) tx_semaphore_put (&aux_uart_1_sema);
  }
  else if ( huart->Instance == AUX_UART_2 )
  {
    (void) tx_semaphore_put (&aux_uart_2_sema);
  }
}

/**
 * @brief  UART error callback.
 * @param  huart UART handle.
 * @retval None
 */
void HAL_UART_ErrorCallback ( UART_HandleTypeDef *huart )
{
  UNUSED(huart);
  // TODO: Add a call to set an error flag here?
}

/**
 * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
void HAL_UARTEx_RxEventCallback ( UART_HandleTypeDef *huart, uint16_t Size )
{
  if ( huart->Instance == CT_UART )
  {
    (void) tx_semaphore_put (&ct_uart_sema);
  }
  else if ( huart->Instance == IRIDIUM_UART )
  {
    (void) tx_semaphore_put (&iridium_uart_sema);
  }
  else if ( huart->Instance == GNSS_UART )
  {
    (void) tx_semaphore_put (&gnss_uart_sema);
  }
  else if ( huart->Instance == AUX_UART_1 )
  {
    (void) tx_semaphore_put (&aux_uart_1_sema);
  }
  else if ( huart->Instance == AUX_UART_2 )
  {
    (void) tx_semaphore_put (&aux_uart_2_sema);
  }
}

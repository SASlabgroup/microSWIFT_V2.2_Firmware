/*
 * interrupt_callbacks.c
 *
 *  Created on: Aug 2, 2024
 *      Author: philbush
 */

#include "spi.h"

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

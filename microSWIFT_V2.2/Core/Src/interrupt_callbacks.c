/*
 * interrupt_callbacks.c
 *
 *  Created on: Aug 2, 2024
 *      Author: philbush
 */

#include "spi.h"
#include "app_threadx.h"
#include "usart.h"
#include "gpio.h"
#include "gnss.h"
#include "battery.h"

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
    (void) tx_event_flags_set (&irq_flags, GNSS_TX_COMPLETE, TX_OR);
  }
//  else if ( huart->Instance == AUX_UART_1 )
//  {
//    (void) tx_semaphore_put (&aux_uart_1_sema);
//  }
//  else if ( huart->Instance == AUX_UART_2 )
//  {
//    (void) tx_semaphore_put (&aux_uart_2_sema);
//  }
  else if ( huart->Instance == LOGGER_UART )
  {
    (void) tx_semaphore_put (&logger_sema);
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
    if ( gnss_get_configured_status () )
    {
      (void) tx_event_flags_set (&irq_flags, GNSS_MSG_RECEIVED, TX_OR);
    }
    else
    {
      (void) tx_event_flags_set (&irq_flags, GNSS_CONFIG_RECVD, TX_OR);
    }
  }
//  else if ( huart->Instance == AUX_UART_1 )
//  {
//    (void) tx_semaphore_put (&aux_uart_1_sema);
//  }
//  else if ( huart->Instance == AUX_UART_2 )
//  {
//    (void) tx_semaphore_put (&aux_uart_2_sema);
//  }
}

/**
 * @brief  UART error callback.
 * @param  huart UART handle.
 * @retval None
 */
void HAL_UART_ErrorCallback ( UART_HandleTypeDef *huart )
{
  uint32_t error_flag = 0;
  UNUSED(huart);
#warning "Add a call to set an error flag here. Probably best to try to manage the error in Control\
          Thread."

  error_flag = HAL_UART_GetError (huart);

  switch ( error_flag )
  {
    case HAL_UART_ERROR_NONE:
      goto not_problem;

    case HAL_UART_ERROR_PE:
      goto problem;

    case HAL_UART_ERROR_NE:
      goto problem;

    case HAL_UART_ERROR_FE:
      goto problem;

    case HAL_UART_ERROR_ORE:
      goto not_problem;

    case HAL_UART_ERROR_DMA:
      goto problem;

    case HAL_UART_ERROR_RTO:
      goto problem;
  }

problem:
  error_flag = 0;

not_problem:
  return;
}

/**
 * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
uint16_t msg_size = 0;
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
    if ( !gnss_get_configured_status () )
    {
      if ( Size == FRAME_SYNC_RX_SIZE )
      {
        (void) tx_event_flags_set (&irq_flags, GNSS_CONFIG_RECVD, TX_OR);
      }
    }
    else if ( Size < UBX_NAV_PVT_MESSAGE_LENGTH )
    {
      msg_size = Size;
      (void) tx_event_flags_set (&irq_flags, GNSS_MSG_INCOMPLETE, TX_OR);
    }
    else
    {
      (void) tx_event_flags_set (&irq_flags, GNSS_MSG_RECEIVED, TX_OR);
    }
  }
//  else if ( huart->Instance == AUX_UART_1 )
//  {
//    (void) tx_semaphore_put (&aux_uart_1_sema);
//  }
//  else if ( huart->Instance == AUX_UART_2 )
//  {
//    (void) tx_semaphore_put (&aux_uart_2_sema);
//  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM16 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 *
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
  // High frequency, low overhead ISR, no need to save/restore context
  if ( htim->Instance == TIM6 )
  {
    HAL_IncTick ();
  }
}

/**
 * @brief  ADC conversion complete callback
 *
 * @param  hadc : ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback ( ADC_HandleTypeDef *hadc )
{
  static uint32_t number_of_conversions = 0;
  static uint64_t v_sum = 0;

  if ( number_of_conversions < NUMBER_OF_ADC_SAMPLES )
  {
    uint32_t sample = HAL_ADC_GetValue (hadc);
    v_sum += (sample * ADC_MICROVOLTS_PER_BIT)
             + HAL_ADCEx_Calibration_GetValue (hadc, ADC_SINGLE_ENDED);
    number_of_conversions++;
  }
  else
  {
    // Write the voltage to the battery struct
    battery_set_voltage (
        (float) ((((float) v_sum / (float) NUMBER_OF_ADC_SAMPLES)
                  + ADC_CALIBRATION_CONSTANT_MICROVOLTS)
                 / MICROVOLTS_PER_VOLT));
    // Reset static variables
    v_sum = 0;
    number_of_conversions = 0;
    // Set the conversion complete flag
    tx_event_flags_set (&irq_flags, BATTERY_CONVERSION_COMPLETE, TX_OR);
    // Done with our samples, shut it down.
    HAL_ADC_Stop_IT (hadc);
  }

}

/**
 * @brief  EXTI line rising detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Rising_Callback ( uint16_t GPIO_Pin )
{
  UNUSED(GPIO_Pin);
}

/**
 * @brief  EXTI line falling detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Falling_Callback ( uint16_t GPIO_Pin )
{
  if ( GPIO_Pin == RTC_INT_A_Pin )
  {
#ifdef DEBUG
    __asm__("BKPT");
#endif
  }
  if ( GPIO_Pin == RTC_INT_B_Pin )
  {
#ifdef DEBUG
    __asm__("BKPT");
#endif
  }

  if ( GPIO_Pin == AS7341_INT_Pin )
  {
    (void) tx_semaphore_put (&light_sensor_int_pin_sema);
  }
}

/**
 * @brief  Memory Tx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MemTxCpltCallback ( I2C_HandleTypeDef *hi2c )
{
  if ( hi2c->Instance == LIGHT_SENSOR_I2C )
  {
    (void) tx_semaphore_put (&light_sensor_i2c_sema);
    (void) tx_semaphore_put (&turbidity_sensor_i2c_sema);
  }
  else if ( hi2c->Instance == TURBIDITY_SENSOR_I2C )
  {
    (void) tx_semaphore_put (&turbidity_sensor_i2c_sema);
  }
}

/**
 * @brief  Memory Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MemRxCpltCallback ( I2C_HandleTypeDef *hi2c )
{
  if ( hi2c->Instance == LIGHT_SENSOR_I2C )
  {
    (void) tx_semaphore_put (&light_sensor_i2c_sema);
    (void) tx_semaphore_put (&turbidity_sensor_i2c_sema);
  }
  else if ( hi2c->Instance == TURBIDITY_SENSOR_I2C )
  {
    (void) tx_semaphore_put (&turbidity_sensor_i2c_sema);
  }
}

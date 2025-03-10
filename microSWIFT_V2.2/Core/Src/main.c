/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include "main.h"
#include "gpdma.h"
#include "lpdma.h"
#include "memorymap.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "testing_hooks.h"
#include "octospi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Configuration bytes programmed using STM32 Cube programmer in the last page of flash
static microSWIFT_configuration flash_config __attribute__ ((section (".uservars.CONFIGURATION")));
Ext_RTC rtc =
  { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config ( void );
static void SystemPower_Config ( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main ( void )
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  // Setup the persistent ram if needed
  persistent_ram_init (&flash_config);
  persistent_ram_increment_sample_window_counter ();

  // Initialize and configure the RTC since we need the 32768 LSE clock signal to calibrate main system clock (MSIS)
  if ( ext_rtc_init (&rtc, device_handles.core_spi_handle, &ext_rtc_spi_sema) != uSWIFT_SUCCESS )
  {
    Error_Handler ();
  }

  // In addition to outputting the clock signal, this will also clear the any active flags/ IRQs (including watchdog)
  if ( rtc.setup_rtc () != uSWIFT_SUCCESS )
  {
    Error_Handler ();
  }

  // Initialize/ configure the watchdog
  if ( rtc.config_watchdog (WATCHDOG_PERIOD) != uSWIFT_SUCCESS )
  {
    Error_Handler ();
  }

  // Shut down flash bank 2 -- no longer required
  HAL_FLASHEx_EnablePowerDown (FLASH_BANK_2);
  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config ();

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */
  // Disable any previous pullups or pull downs
  HAL_PWREx_DisablePullUpPullDownConfig ();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_GPDMA1_Init ();
  MX_LPDMA1_Init ();
  /* USER CODE BEGIN 2 */

  tests_init ();

  if ( tests.main_test != NULL )
  {
    tests.main_test (NULL);
  }

  /* USER CODE END 2 */

  MX_ThreadX_Init ();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while ( 1 )
  {
    HAL_NVIC_SystemReset ();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config ( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

  /** Configure the main internal regulator output voltage
   */
  if ( HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK )
  {
    Error_Handler ();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI
                                     | RCC_OSCILLATORTYPE_MSIK;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
  RCC_OscInitStruct.MSIKClockRange = RCC_MSIKRANGE_2;
  RCC_OscInitStruct.MSIKState = RCC_MSIK_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if ( HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK )
  {
    Error_Handler ();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if ( HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK )
  {
    Error_Handler ();
  }

  /** Enable MSI Auto calibration
   */
  HAL_RCCEx_EnableMSIPLLModeSelection (RCC_MSIKPLL_MODE_SEL);
  HAL_RCCEx_EnableMSIPLLMode ();
}

/**
 * @brief Power Configuration
 * @retval None
 */
static void SystemPower_Config ( void )
{
  HAL_PWREx_EnableVddIO2 ();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if ( HAL_PWREx_ConfigSupply (PWR_SMPS_SUPPLY) != HAL_OK )
  {
    Error_Handler ();
  }
  /* USER CODE BEGIN PWR */
  /* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if ( htim->Instance == TIM6 )
  {
    HAL_IncTick ();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler ( void )
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq ();
  while ( 1 )
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed ( uint8_t *file, uint32_t line )
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

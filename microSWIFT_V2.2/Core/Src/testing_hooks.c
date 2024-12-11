/*
 * testing_hooks.c
 *
 *  Created on: Aug 1, 2024
 *      Author: philbush
 */

#include "testing_hooks.h"
#include "stddef.h"
#include "ext_rtc.h"
#include "lptim.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
bool test_lptim_crytstal ( void *unused );
/**************************************************************************************************/
/*********************************** Init --> Assign Tests ****************************************/
/**************************************************************************************************/
void tests_init ( void )
{
  tests.main_test = NULL;
  tests.threadx_init_test = NULL;
  tests.control_test = NULL;
  tests.gnss_thread_test = NULL;
  tests.ct_thread_test = NULL;
  tests.light_thread_test = NULL;
  tests.turbidity_thread_test = NULL;
  tests.accelerometer_thread_test = NULL;
  tests.expansion_thread_1_test = NULL;
  tests.expansion_thread_2_test = NULL;
  tests.expansion_thread_3_test = NULL;
  tests.waves_thread_test = NULL;
  tests.iridium_thread_test = NULL;
  tests.shutdown_test = NULL;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Definitions ############################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

bool test_lptim_crytstal ( void *unused )
{
  HAL_LPTIM_Counter_Start_IT (&hlptim1);
  return true;
}

void HAL_LPTIM_CompareMatchCallback ( LPTIM_HandleTypeDef *hlptim )
{

  static bool toggle = false;

  HAL_GPIO_WritePin (GPIOF, GPIO_PIN_13, toggle);
  toggle = !toggle;

  HAL_LPTIM_Counter_Start_IT (&hlptim1);

}

void HAL_LPTIM_AutoReloadMatchCallback ( LPTIM_HandleTypeDef *hlptim )
{

  static bool toggle = false;

  HAL_GPIO_WritePin (GPIOF, GPIO_PIN_13, toggle);
  toggle = !toggle;

  HAL_LPTIM_Counter_Start_IT (&hlptim1);
}

void HAL_LPTIM_TriggerCallback ( LPTIM_HandleTypeDef *hlptim )
{

  static bool toggle = false;

  HAL_GPIO_WritePin (GPIOF, GPIO_PIN_13, toggle);
  toggle = !toggle;

  HAL_LPTIM_Counter_Start_IT (&hlptim1);
}

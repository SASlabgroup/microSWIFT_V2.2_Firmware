/*
 * testing_hooks.c
 *
 *  Created on: Aug 1, 2024
 *      Author: philbush
 */

#include "testing_hooks.h"
#include "stddef.h"
#include "ext_rtc.h"
#include "app_threadx.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

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
  tests.waves_thread_test = NULL;
  tests.iridium_thread_test = NULL;
  tests.filex_test = NULL;
  tests.shutdown_test = NULL;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Definitions ############################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
bool filex_test ( void *unused )
{

}

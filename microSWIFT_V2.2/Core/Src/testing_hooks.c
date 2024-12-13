/*
 * testing_hooks.c
 *
 *  Created on: Aug 1, 2024
 *      Author: philbush
 */

#include "testing_hooks.h"
#include "stddef.h"
#include "ext_rtc.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ext_psram.h"
bool test_psram ( void *unused );
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

bool test_psram ( void *unused )
{
  const uint8_t test_str[] = "This is a test of the external PSRAM chip.";
  uint8_t read_buf[64] =
    { 0 };
  uint8_t data_len = strlen (test_str);
  uint8_t *psram_addr = (uint8_t*) OCTOSPI1_BASE;

  if ( !initialize_psram () )
  {
    return false;
  }

  // Test write
  for ( int i = 0; i < data_len; i++, psram_addr++ )
  {
    *psram_addr = test_str[i];
  }

  psram_addr = (uint8_t*) OCTOSPI1_BASE;

  // Test read
  for ( int i = 0; i < data_len; i++ )
  {
    read_buf[i] = *psram_addr;
  }

  return (strcmp (test_str, read_buf) == 0);
}

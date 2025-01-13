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
#include "gnss.h"
#include "file_system_server.h"
#include "math.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
bool filex_test_gnss ( void *gnss_ptr );
/**************************************************************************************************/
/*********************************** Init --> Assign Tests ****************************************/
/**************************************************************************************************/
void tests_init ( void )
{
  tests.main_test = NULL;
  tests.threadx_init_test = NULL;
  tests.control_test = NULL;
  tests.gnss_thread_test = filex_test_gnss;
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
bool filex_test_gnss ( void *gnss_ptr )
{
  GNSS *gnss = (GNSS*) gnss_ptr;
  double sin_constant;

  gnss->sample_window_start_time = 1736531700U;

  for ( int i = 0; i < gnss->global_config->samples_per_window; i++ )
  {
    sin_constant = (i + 1.0f) / 31.428f;

    gnss->GNSS_N_Array[i] = 10.0f * sin (sin_constant);
    gnss->GNSS_E_Array[i] = 10.0f * sin (sin_constant + 1);
    gnss->GNSS_D_Array[i] = 10.0f * sin (sin_constant + 2);

    if ( (i % gnss->global_config->gnss_sampling_rate) == 0 )
    {
      gnss->breadcrumb_track[gnss->breadcrumb_index].lat = 47.65 + (((float) i) / 100000.0f);
      gnss->breadcrumb_track[gnss->breadcrumb_index].lon = -122.48 + (((float) i) / 100000.0f);
      gnss->breadcrumb_index++;
    }

    gnss->total_samples++;
  }

  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

  if ( file_system_server_save_gnss_raw (gnss) != uSWIFT_SUCCESS )
  {
    return false;
  }

  if ( file_system_server_save_gnss_track (gnss) != uSWIFT_SUCCESS )
  {
    return false;
  }

  return true;
}

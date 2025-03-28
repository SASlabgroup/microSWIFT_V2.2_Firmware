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
#include "iridium.h"
#include "logger.h"
#include "file_system_server.h"
#include "math.h"

testing_hooks tests;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Declarations ###########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
bool test_iridium_queueing ( void *iridium_ptr );
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
  tests.iridium_thread_test = test_iridium_queueing;
  tests.filex_test = NULL;
  tests.shutdown_test = NULL;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Test Definitions ############################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
bool test_iridium_queueing ( void *iridium_ptr )
{
  Iridium *iridium = (Iridium*) iridium_ptr;
  uSWIFT_return_code_t ret;
  bool current_message_sent = false;
  uint32_t next_message_type = NO_MESSAGE, next_message_size = 0;
  char *log_str;
  sbd_message_type_52 sbd_message =
    { 0 };
  sbd_message_type_54 light_message =
    { 0 };
  uint8_t *msg_ptr;
  real16_T E[42];
  real16_T Dp;
  real16_T Hs;
  real16_T Tp;
  real16_T b_fmax;
  real16_T b_fmin;
  signed char a1[42];
  signed char a2[42];
  signed char b1[42];
  signed char b2[42];
  unsigned char check[42];

  Dp.bitPattern = TELEMETRY_FIELD_ERROR_CODE;
  Tp.bitPattern = TELEMETRY_FIELD_ERROR_CODE;
  b_fmax.bitPattern = TELEMETRY_FIELD_ERROR_CODE;
  b_fmin.bitPattern = TELEMETRY_FIELD_ERROR_CODE;
  for ( int i = 0; i < 42; i++ )
  {
    E[i].bitPattern = TELEMETRY_FIELD_ERROR_CODE;
    a1[i] = TELEMETRY_FIELD_ERROR_CODE;
    a2[i] = TELEMETRY_FIELD_ERROR_CODE;
    b1[i] = TELEMETRY_FIELD_ERROR_CODE;
    b2[i] = TELEMETRY_FIELD_ERROR_CODE;
    check[i] = TELEMETRY_FIELD_ERROR_CODE;
  }
  memcpy (&sbd_message.Tp, &Tp, sizeof(real16_T));
  memcpy (&sbd_message.Dp, &Dp, sizeof(real16_T));
  memcpy (&(sbd_message.E_array[0]), &(E[0]), 42 * sizeof(real16_T));
  memcpy (&sbd_message.f_min, &b_fmin, sizeof(real16_T));
  memcpy (&sbd_message.f_max, &b_fmax, sizeof(real16_T));
  memcpy (&(sbd_message.a1_array[0]), &(a1[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.b1_array[0]), &(b1[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.a2_array[0]), &(a2[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.b2_array[0]), &(b2[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.cf_array[0]), &(check[0]), 42 * sizeof(unsigned char));

  // Make a bunch of type 52 messages, half with error values, half with an increasing
  // significant wave height
  for ( int i = 0; i < 10; i++ )
  {
    Hs.bitPattern = (i % 2 == 0) ?
        TELEMETRY_FIELD_ERROR_CODE : 0x4248 + i;
    memcpy (&sbd_message.Hs, &Hs, sizeof(real16_T));

    persistent_ram_save_message (WAVES_TELEMETRY, &sbd_message);
  }

  for ( int i = 0; i < 10 * LIGHT_MSGS_PER_SBD; i++ )
  {
    memset (&light_message.start_lat, i + 1, sizeof(uint8_t));
    memset (&light_message.start_lon, i + 1, sizeof(uint8_t));
    memset (&light_message.end_lat, i + 1, sizeof(uint8_t));
    memset (&light_message.end_lon, i + 1, sizeof(uint8_t));
    memset (&light_message.start_timestamp, i + 1, sizeof(uint8_t));
    memset (&light_message.end_timestamp, i + 1, sizeof(uint8_t));
    memset (&light_message.max_reading_clear, i + 1, sizeof(uint8_t));
    memset (&light_message.min_reading_clear, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_clear, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f1, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f2, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f3, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f4, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f5, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f6, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f7, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_f8, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_dark, i + 1, sizeof(uint8_t));
    memset (&light_message.avg_nir, i + 1, sizeof(uint8_t));

    persistent_ram_save_message (LIGHT_TELEMETRY, &light_message);
  }

  watchdog_check_in (IRIDIUM_THREAD);

  while ( 1 )
  {
    if ( !current_message_sent )
    {
      LOG("Attempting transmission of NEDWaves telemetry...");
      ret = iridium.transmit_message (&(msg_buffer[0]), sizeof(sbd_message_type_52));
      if ( ret == uSWIFT_SUCCESS )
      {
        current_message_sent = true;
      }
      else if ( ret == uSWIFT_TIMEOUT )
      {
        break;
      }

      continue;
    }

    next_message_type = get_next_telemetry_message (&msg_ptr, &configuration);
    if ( next_message_type == NO_MESSAGE )
    {
      LOG("No messages messages left in cache.");
      break;
    }

    next_message_size = (next_message_type == WAVES_TELEMETRY) ?
        sizeof(sbd_message_type_52) :
                        (next_message_type == TURBIDITY_TELEMETRY) ?
                            sizeof(sbd_message_type_53) :
                        (next_message_type == LIGHT_TELEMETRY) ?
                            sizeof(sbd_message_type_54) : 0;

    if ( next_message_size > 0 )
    {
      log_str = (next_message_type == WAVES_TELEMETRY) ?
          "NEDWaves" :
                (next_message_type == TURBIDITY_TELEMETRY) ?
                    "OBS" :
                (next_message_type == LIGHT_TELEMETRY) ?
                    "Light" : "Unknown";
      LOG("Attempting transmission of %s telemetry...", log_str);

      if ( iridium.transmit_message (msg_ptr, next_message_size) == uSWIFT_SUCCESS )
      {
        persistent_ram_delete_message_element (next_message_type, msg_ptr);
      }
    }
  }
}

/*
 * persistent_ram.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include "persistent_ram.h"
#include "math.h"
#include "string.h"
#include "time.h"
#include "ext_rtc_api.h"

// Save the struct in SRAM2 -- NOLOAD section which will be retained in standby mode
static Persistent_Storage persistent_self __attribute__((section(".sram2")));

// Helper functions
static void _persistent_ram_clear ( void );
static void close_out_error_msg ( uint32_t msg_index );

void persistent_ram_init ( void )
{
  // The ram section "persistent_self" resides in is a NOLOAD section -- the contents reset and standby mode
  // once set.
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
  }
}

void persistent_ram_deinit ( void )
{
  // Clear everything out
  _persistent_ram_clear ();
}

void persistent_ram_increment_sample_window_counter ( void )
{
  persistent_self.sample_window_counter++;
}

int32_t persistent_ram_get_sample_window_counter ( void )
{
  return persistent_self.sample_window_counter;
}

void persistent_ram_save_iridium_message ( sbd_message_type_52 *msg )
{
  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
  }

  // If the storage queue is full, see if we can replace an element
  if ( persistent_self.telemetry_storage.num_telemetry_msgs_enqueued == MAX_NUM_IRIDIUM_MSGS_STORED )
  {
    for ( int i = 0; i < MAX_NUM_IRIDIUM_MSGS_STORED; i++ )
    {
      // Want to make sure we are comparing absolute values, though that should never be the case...
      if ( (fabsf (halfToFloat (persistent_self.telemetry_storage.msg_queue[i].payload.Hs)))
           < fabsf (halfToFloat (msg->Hs)) )
      {
        // copy the message over
        memcpy (&(persistent_self.telemetry_storage.msg_queue[i].payload), msg,
                sizeof(sbd_message_type_52));
        // Make the entry valid (should already be, but just in case)
        persistent_self.telemetry_storage.msg_queue[i].valid = true;
        return;
      }
    }
  }
  else
  {
    // Queue is not full, just need to find an open slot
    for ( int i = 0; i < MAX_NUM_IRIDIUM_MSGS_STORED; i++ )
    {
      if ( !persistent_self.telemetry_storage.msg_queue[i].valid )
      {
        // copy the message over
        memcpy (&(persistent_self.telemetry_storage.msg_queue[i].payload), msg,
                sizeof(sbd_message_type_52));
        // Make the entry valid
        persistent_self.telemetry_storage.msg_queue[i].valid = true;
        persistent_self.telemetry_storage.num_telemetry_msgs_enqueued++;
        return;
      }
    }
  }
}

void persistent_ram_log_error_string ( char *error_str )
{
  uint32_t error_str_len = strlen (error_str);
  struct tm timestamp =
    { 0 };
  char timestamp_str[32] =
    { 0 };

  // If there is not enough space in the current message, close out the current msg and grab a new one
  if ( (persistent_self.error_storage.char_buf_index + TIMESTAMP_STR_LEN + error_str_len + 2)
       > TYPE_99_CHAR_BUF_LEN )
  {
    close_out_error_msg (persistent_self.error_storage.current_msg_index);

    if ( persistent_self.error_storage.num_error_msgs_enqueued == MAX_NUM_ERROR_MSGS_STORED )
    {
      return;
    }
    else
    {
      for ( int i = 0; i < MAX_NUM_ERROR_MSGS_STORED; i++ )
      {
        if ( persistent_self.error_storage.msg_queue[i].state == ERROR_MSG_EMPTY )
        {
          persistent_self.error_storage.current_msg_index = i;
          persistent_self.error_storage.char_buf_index = 0;
          break;
        }

      }
    }
  }

  // Mark the message as in use if it is not already (new message from queue)
  if ( persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index]
      .state
       == ERROR_MSG_EMPTY )
  {
    persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index].state =
        ERROR_MSG_IN_USE;
  }
  // get a timestamp from rtc
  if ( rtc_server_get_time (&timestamp, PERSISTENT_RAM_REQUEST_PROCESSED) != uSWIFT_SUCCESS )
  {
    return;
  }
  // Convert timestamp to string
  if ( strftime (&timestamp_str[0], sizeof(timestamp_str), TIMESTAMP_STR_FORMAT,
                 &timestamp) != TIMESTAMP_STR_LEN )
  {
    return;
  }
  // Write the timestamp to the error msg char buf
  memcpy (
      &persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index]
          .payload.char_buf[persistent_self.error_storage.char_buf_index],
      &timestamp_str[0], TIMESTAMP_STR_LEN);
  // adjust the char buffer index
  persistent_self.error_storage.char_buf_index += TIMESTAMP_STR_LEN;
  // Write the error str
  memcpy (
      &persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index]
          .payload.char_buf[persistent_self.error_storage.char_buf_index],
      error_str, error_str_len);
  // adjust the char buffer index
  persistent_self.error_storage.char_buf_index += error_str_len;
  // Add a breakline
  memcpy (
      &persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index]
          .payload.char_buf[persistent_self.error_storage.char_buf_index],
      "\n", 1);
  // adjust the char buffer index
  persistent_self.error_storage.char_buf_index += 1;
}

sbd_message_type_52* persistent_ram_get_prioritized_unsent_iridium_message ( void )
{
  float most_significant_wave_height = 0.0;
  float msg_wave_height = 0.0;
  real16_T msg_wave_half_float =
    { 0 };
  sbd_message_type_52 *ret_ptr = NULL;
  int32_t msg_index = 0;

  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return ret_ptr;
  }

  // Empty queue check
  if ( persistent_self.telemetry_storage.num_telemetry_msgs_enqueued == 0 )
  {
    return ret_ptr;
  }

  // Find the largest significant wave height
  for ( int i = 0; i < MAX_NUM_IRIDIUM_MSGS_STORED; i++ )
  {
    if ( persistent_self.telemetry_storage.msg_queue[i].valid )
    {
      msg_wave_half_float = persistent_self.telemetry_storage.msg_queue[i].payload.Hs;
      msg_wave_height = fabsf (halfToFloat (msg_wave_half_float));

      if ( msg_wave_height > most_significant_wave_height )
      {
        most_significant_wave_height = msg_wave_height;
        msg_index = i;
      }
    }
  }

  // Make sure we don't go out of bounds
  if ( (msg_index > 0) && (msg_index < (MAX_NUM_IRIDIUM_MSGS_STORED - 1)) )
  {
    ret_ptr = &persistent_self.telemetry_storage.msg_queue[msg_index].payload;
  }

  return ret_ptr;
}

sbd_message_type_99* persistent_ram_get_prioritized_unsent_error_message ( void )
{
  time_t most_recent = 0, this_msg = 0;
  sbd_message_type_99 *ret_ptr = NULL;
  int32_t msg_index = 0;

  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return ret_ptr;
  }

  // If the current message is in use, send it first
  if ( persistent_self.error_storage.msg_queue[persistent_self.error_storage.current_msg_index]
      .state
       == ERROR_MSG_IN_USE )
  {
    ret_ptr = &persistent_self.error_storage.msg_queue[persistent_self.error_storage
        .current_msg_index].payload;
    return ret_ptr;
  }
  // Empty queue check
  else if ( persistent_self.error_storage.num_error_msgs_enqueued == 0 )
  {
    return ret_ptr;
  }

  // Otherwise, find the most recent message
  for ( int i = 0; i < MAX_NUM_ERROR_MSGS_STORED; i++ )
  {
    if ( persistent_self.error_storage.msg_queue[i].state == ERROR_MSG_FULL )
    {
      this_msg = persistent_self.error_storage.msg_queue[i].payload.timetamp;

      if ( this_msg > most_recent )
      {
        most_recent = this_msg;
        msg_index = i;
      }
    }
  }

  // Make sure we don't go out of bounds
  if ( (msg_index > 0) && (msg_index < (MAX_NUM_IRIDIUM_MSGS_STORED - 1)) )
  {
    ret_ptr = &persistent_self.error_storage.msg_queue[msg_index].payload;
  }

  return ret_ptr;
}

void persistent_ram_delete_iridium_message_element ( sbd_message_type_52 *msg_ptr )
{
  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return;
  }

  // Find the pointer
  for ( int i = 0; i < MAX_NUM_IRIDIUM_MSGS_STORED; i++ )
  {
    if ( &persistent_self.telemetry_storage.msg_queue[i].payload == msg_ptr )
    {
      // copy the message over
      memset (&(persistent_self.telemetry_storage.msg_queue[i].payload), 0,
              sizeof(sbd_message_type_52));
      // Make the entry valid
      persistent_self.telemetry_storage.msg_queue[i].valid = false;
      persistent_self.telemetry_storage.num_telemetry_msgs_enqueued--;
      return;
    }
  }
}

void persistent_ram_delete_error_message_element ( sbd_message_type_99 *msg_ptr )
{
  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return;
  }

  // Find the pointer
  for ( int i = 0; i < MAX_NUM_ERROR_MSGS_STORED; i++ )
  {
    if ( &persistent_self.error_storage.msg_queue[i].payload == msg_ptr )
    {
      // copy the message over
      memset (&(persistent_self.error_storage.msg_queue[i].payload), 0,
              sizeof(sbd_message_type_99));
      // Make the entry valid
      persistent_self.error_storage.msg_queue[i].state = ERROR_MSG_EMPTY;
      persistent_self.error_storage.num_error_msgs_enqueued--;
      return;
    }
  }
}

static void _persistent_ram_clear ( void )
{
  // Clear everything out
  persistent_self.sample_window_counter = 0;
  memset (&persistent_self.telemetry_storage, 0, sizeof(Telemetry_Message_Storage));
  memset (&persistent_self.error_storage, 0, sizeof(Error_Message_Storage));
  persistent_self.magic_number = PERSISTENT_RAM_MAGIC_DOUBLE_WORD;
}

static void close_out_error_msg ( uint32_t msg_index )
{
  /*
   * Null terminate msg
   * Add timestamp, lat/lon?
   *
   */
#error "Fill this out before testing."

  return;
}

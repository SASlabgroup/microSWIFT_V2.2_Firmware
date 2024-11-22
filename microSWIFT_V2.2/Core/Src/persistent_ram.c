/*
 * persistent_ram.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include <ext_rtc_server.h>
#include "persistent_ram.h"
#include "math.h"
#include "string.h"
#include "time.h"

// Save the struct in SRAM2 -- NOLOAD section which will be retained in standby mode
static Persistent_Storage persistent_self __attribute__((section(".sram2")));

// Helper functions
static void _persistent_ram_clear ( void );

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

static void _persistent_ram_clear ( void )
{
  // Clear everything out
  persistent_self.sample_window_counter = 0;
  memset (&persistent_self.telemetry_storage, 0, sizeof(Telemetry_Message_Storage));
  persistent_self.magic_number = PERSISTENT_RAM_MAGIC_DOUBLE_WORD;
}

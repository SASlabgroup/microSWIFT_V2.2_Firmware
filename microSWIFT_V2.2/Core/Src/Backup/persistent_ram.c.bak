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

/**
 * Initialize the persistent memory (SRAM 2). This memory section can be maintained during Standby mode.
 *
 * @return void
 */
void persistent_ram_init ( void )
{
  // The ram section "persistent_self" resides in is a NOLOAD section -- the contents reset and standby mode
  // once set.
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
  }
}

/**
 * Deinitialize the persistent memory (SRAM 2), clearing all contents.
 *
 * @return void
 */
void persistent_ram_deinit ( void )
{
  // Clear everything out
  _persistent_ram_clear ();
}

/**
 * Increment the duty cycle counter.
 *
 * @return void
 */
void persistent_ram_increment_sample_window_counter ( void )
{
  persistent_self.sample_window_counter++;
}

/**
 * Return the duty cycle count (counting up from power on).
 *
 * @return void
 */
int32_t persistent_ram_get_sample_window_counter ( void )
{
  return persistent_self.sample_window_counter;
}

/**
 * Save an SBD message for later transmission.
 * !! Turbidity and Light messages are saved as single elements, not a full SBD message.
 *
 * @return void
 */
void persistent_ram_save_message ( telemetry_type_t msg_type, uint8_t *msg )
{
  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
  }

  switch ( msg_type )
  {
    case WAVES_TELEMETRY:
      // If the storage queue is full, see if we can replace an element
      if ( persistent_self.waves_storage.num_telemetry_msgs_enqueued == MAX_NUM_WAVES_MSGS_STORED )
      {
        for ( int i = 0; i < MAX_NUM_WAVES_MSGS_STORED; i++ )
        {
          // Want to make sure we are comparing absolute values, though wave height should always be positive
          if ( (fabsf (halfToFloat (persistent_self.waves_storage.msg_queue[i].payload.Hs)))
               < fabsf (halfToFloat (((sbd_message_type_52*) msg)->Hs)) )
          {
            // copy the message over
            memcpy (&(persistent_self.waves_storage.msg_queue[i].payload), msg,
                    sizeof(sbd_message_type_52));
            // Make the entry valid (should already be, but just in case)
            persistent_self.waves_storage.msg_queue[i].valid = true;
            return;
          }
        }
      }
      else
      {
        // Not full, just need to find an open slot
        for ( int i = 0; i < MAX_NUM_WAVES_MSGS_STORED; i++ )
        {
          if ( !persistent_self.waves_storage.msg_queue[i].valid )
          {
            // copy the message over
            memcpy (&(persistent_self.waves_storage.msg_queue[i].payload), msg,
                    sizeof(sbd_message_type_52));
            // Make the entry valid
            persistent_self.waves_storage.msg_queue[i].valid = true;
            persistent_self.waves_storage.num_telemetry_msgs_enqueued++;
            return;
          }
        }
      }

      break;

    case TURBIDITY_TELEMETRY:
      // If the storage queue is full, then just skip
      if ( !(persistent_self.turbidity_storage.num_msg_elements_enqueued
             == MAX_NUM_TURBIDITY_MSGS_STORED) )
      {
        for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
        {

          for ( int j = 0; j < TURBIDITY_MSGS_PER_SBD; j++ )
          {

            if ( !persistent_self.turbidity_storage.msg_queue[i].valid[j] )
            {
              // copy the message over
              memcpy (&(persistent_self.turbidity_storage.msg_queue[i].payload), msg,
                      sizeof(sbd_message_type_60_element));
              // Make the entry valid
              persistent_self.turbidity_storage.msg_queue[i].valid[j] = true;
              persistent_self.turbidity_storage.num_msg_elements_enqueued++;
              return;
            }
          }
        }
      }

      break;

    case LIGHT_TELEMETRY:
      // If the storage queue is full, then just skip
      if ( !(persistent_self.light_storage.num_msg_elements_enqueued == MAX_NUM_LIGHT_MSGS_STORED) )
      {
        for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
        {
          for ( int j = 0; j < LIGHT_MSGS_PER_SBD; j++ )
          {
            if ( !persistent_self.light_storage.msg_queue[i].valid[j] )
            {
              // copy the message over
              memcpy (&(persistent_self.light_storage.msg_queue[i].payload), msg,
                      sizeof(sbd_message_type_61_element));
              // Make the entry valid
              persistent_self.light_storage.msg_queue[i].valid[j] = true;
              persistent_self.light_storage.num_msg_elements_enqueued++;
              return;
            }
          }
        }
      }

      break;

    default:
      return;
  }
}

/**
 * Return a pointer to the highest priority message of a given type. If no message is
 * available, return pointer will be NULL.
 * !!@ This always returns a pointer to a FULL SBD MESSAGE, not just a single message element.
 *
 * @return void
 */
uint8_t* persistent_ram_get_prioritized_unsent_message ( telemetry_type_t msg_type )
{
  float most_significant_wave_height = 0.0;
  float msg_wave_height = 0.0;
  real16_T msg_wave_half_float =
    { 0 };
  int32_t msg_index = 0;
  bool msg_full = false;
  uint8_t *ret_ptr = NULL;

  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return NULL;
  }

  switch ( msg_type )
  {
    case WAVES_TELEMETRY:

      // Empty queue check
      if ( persistent_self.waves_storage.num_telemetry_msgs_enqueued == 0 )
      {
        return NULL;
      }

      // Find the largest significant wave height
      for ( int i = 0; i < MAX_NUM_WAVES_MSGS_STORED; i++ )
      {
        if ( persistent_self.waves_storage.msg_queue[i].valid )
        {
          msg_wave_half_float = persistent_self.waves_storage.msg_queue[i].payload.Hs;
          msg_wave_height = fabsf (halfToFloat (msg_wave_half_float));

          if ( msg_wave_height > most_significant_wave_height )
          {
            most_significant_wave_height = msg_wave_height;
            msg_index = i;
          }
        }
      }

      // Make sure we don't go out of bounds
      if ( (msg_index > 0) && (msg_index < (MAX_NUM_WAVES_MSGS_STORED - 1)) )
      {
        ret_ptr = (uint8_t*) &persistent_self.waves_storage.msg_queue[msg_index].payload;
      }

      break;

    case TURBIDITY_TELEMETRY:

      // Make sure we have enough elements to constitute a full msg
      if ( persistent_self.turbidity_storage.num_msg_elements_enqueued < TURBIDITY_MSGS_PER_SBD )
      {
        return NULL;
      }

      // Just take the first available
      for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
      {

        msg_full = true;

        for ( int j = 0; j < TURBIDITY_MSGS_PER_SBD; j++ )
        {
          if ( !persistent_self.turbidity_storage.msg_queue[i].valid[j] )
          {
            msg_full = false;
            break;
          }
        }

        if ( msg_full )
        {
          ret_ptr = (uint8_t*) &persistent_self.turbidity_storage.msg_queue[i].payload;
        }
      }

      break;

    case LIGHT_TELEMETRY:

      // Make sure we have enough elements to constitute a full msg
      if ( persistent_self.light_storage.num_msg_elements_enqueued < LIGHT_MSGS_PER_SBD )
      {
        return NULL;
      }

      // Just take the first available
      for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
      {

        msg_full = true;

        for ( int j = 0; j < LIGHT_MSGS_PER_SBD; j++ )
        {
          if ( !persistent_self.light_storage.msg_queue[i].valid[j] )
          {
            msg_full = false;
            break;
          }
        }

        if ( msg_full )
        {
          ret_ptr = (uint8_t*) &persistent_self.light_storage.msg_queue[i].payload;
        }
      }

      break;

    default:
      ret_ptr = NULL;
  }

  return ret_ptr;
}

void persistent_ram_delete_message_element ( telemetry_type_t msg_type, uint8_t *msg_ptr )
{
  // Corruption, lack of initialization check
  if ( persistent_self.magic_number != PERSISTENT_RAM_MAGIC_DOUBLE_WORD )
  {
    _persistent_ram_clear ();
    return;
  }

  switch ( msg_type )
  {
    case WAVES_TELEMETRY:

      // Find the pointer
      for ( int i = 0; i < MAX_NUM_WAVES_MSGS_STORED; i++ )
      {
        if ( (uint8_t*) &persistent_self.waves_storage.msg_queue[i].payload == msg_ptr )
        {
          // Zero out the message
          memset (msg_ptr, 0, sizeof(Iridium_Message_Storage_Element_t));
          persistent_self.waves_storage.num_telemetry_msgs_enqueued--;
        }
      }

      break;

    case TURBIDITY_TELEMETRY:

      // Find the pointer
      for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
      {
        if ( (uint8_t*) &persistent_self.turbidity_storage.msg_queue[i].payload == msg_ptr )
        {
          // Zero out the message
          memset (msg_ptr, 0, sizeof(Turbidity_Message_Storage_Element_t));
          persistent_self.turbidity_storage.num_msg_elements_enqueued -= TURBIDITY_MSGS_PER_SBD;
        }
      }

      break;

    case LIGHT_TELEMETRY:

      // Find the pointer
      for ( int i = 0; i < MAX_NUM_NON_WAVES_MSGS_STORED; i++ )
      {
        if ( (uint8_t*) &persistent_self.light_storage.msg_queue[i].payload == msg_ptr )
        {
          // Zero out the message
          memset (msg_ptr, 0, sizeof(Light_Message_Storage_Element_t));
          persistent_self.light_storage.num_msg_elements_enqueued -= LIGHT_MSGS_PER_SBD;
        }
      }

      break;

    default:
      break;
  }
}

static void _persistent_ram_clear ( void )
{
  // Clear everything out
  memset (&persistent_self, 0, sizeof(Persistent_Storage));
  persistent_self.magic_number = PERSISTENT_RAM_MAGIC_DOUBLE_WORD;
}

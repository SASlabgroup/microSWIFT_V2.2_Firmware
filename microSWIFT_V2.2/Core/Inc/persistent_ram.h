/*
 * persistent_ram.h
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#ifndef INC_PERSISTENT_RAM_H_
#define INC_PERSISTENT_RAM_H_

#include "stdbool.h"
#include "stddef.h"
#include "iridium.h"
#include "NEDWaves/rtwhalf.h"

#define PERSISTENT_RAM_MAGIC_DOUBLE_WORD 0x5048494C42555348
#define MAX_NUM_IRIDIUM_MSGS_STORED (24U * 7U)
#define MAX_NUM_ERROR_MSGS_STORED   (24u)

// @formatter:off
typedef struct
{
  sbd_message_type_52       payload;
  bool                      valid;
} Iridium_Message_t;

typedef struct
{
  sbd_message_type_99   payload;
  bool                  valid;
} Iridium_Error_Message_t;


typedef struct
{
  Iridium_Message_t         msg_queue[MAX_NUM_IRIDIUM_MSGS_STORED];
  Iridium_Error_Message_t   error_msg_queue[MAX_NUM_ERROR_MSGS_STORED];
  uint32_t                  num_telemetry_msgs_enqueued;
  uint32_t                  num_error_msgs_enqueued;
} Iridium_Message_Storage;

// All of the things we need to survive in standby mode
typedef struct
{
  uint64_t                  magic_number;
  int32_t                   sample_window_counter;
  Iridium_Message_Storage   unsent_msg_storage;
} Persistent_Storage;

void                    persistent_ram_init ( void );
void                    persistent_ram_deinit ( void );
void                    persistent_ram_increment_sample_window_counter ( void );
int32_t                 persistent_ram_get_sample_window_counter ( void );
void                    persistent_ram_save_iridium_message ( sbd_message_type_52* msg );
void                    persistent_ram_save_error_message (sbd_message_type_99* msg);
sbd_message_type_52*    persistent_ram_get_prioritized_unsent_iridium_message ( void );
sbd_message_type_99*    persistent_ram_get_prioritized_unsent_error_message ( void );
void                    persistent_ram_delete_iridium_message_element ( sbd_message_type_52 *msg_ptr );
void                    persistent_ram_delete_error_message_element ( sbd_message_type_99 *msg_ptr );
// @formatter:on

#endif /* INC_PERSISTENT_RAM_H_ */

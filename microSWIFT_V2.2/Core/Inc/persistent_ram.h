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
#include "sbd.h"
#include "NEDWaves/rtwhalf.h"

#define PERSISTENT_RAM_MAGIC_DOUBLE_WORD 0x5048494C42555348
#define MAX_NUM_IRIDIUM_MSGS_STORED (24U * 7U)
#define MAX_NUM_ERROR_MSGS_STORED   (24U)
#define TIMESTAMP_STR_FORMAT "%m/%d/%y %H:%M:%S"
#define TIMESTAMP_STR_LEN (17U)
// @formatter:off
typedef struct
{
  sbd_message_type_52       payload;
  bool                      valid;
} Iridium_Message_t;

typedef enum
{
  ERROR_MSG_EMPTY     = 0,
  ERROR_MSG_IN_USE    = 1,
  ERROR_MSG_FULL      = 2
} error_msg_state_t;

typedef struct
{
  sbd_message_type_99   payload;
  error_msg_state_t     state;
} Iridium_Error_Message_t;

typedef struct
{
  Iridium_Message_t         msg_queue[MAX_NUM_IRIDIUM_MSGS_STORED];
  uint32_t                  num_telemetry_msgs_enqueued;
} Telemetry_Message_Storage;

typedef struct
{
  Iridium_Error_Message_t   msg_queue[MAX_NUM_ERROR_MSGS_STORED];
  uint32_t                  num_error_msgs_enqueued;
  uint32_t                  current_msg_index;
  uint32_t                  char_buf_index;
} Error_Message_Storage;

// All of the things we need to survive in standby mode
typedef struct
{
  uint64_t                  magic_number;
  int32_t                   sample_window_counter;
  Telemetry_Message_Storage telemetry_storage;
  Error_Message_Storage     error_storage;
} Persistent_Storage;

void                    persistent_ram_init ( void );
void                    persistent_ram_deinit ( void );
void                    persistent_ram_increment_sample_window_counter ( void );
int32_t                 persistent_ram_get_sample_window_counter ( void );
void                    persistent_ram_save_iridium_message ( sbd_message_type_52* msg );
void                    persistent_ram_log_error_string ( char* error_str );
sbd_message_type_52*    persistent_ram_get_prioritized_unsent_iridium_message ( void );
sbd_message_type_99*    persistent_ram_get_prioritized_unsent_error_message ( void );
void                    persistent_ram_delete_iridium_message_element ( sbd_message_type_52 *msg_ptr );
void                    persistent_ram_delete_error_message_element ( sbd_message_type_99 *msg_ptr );
// @formatter:on

#endif /* INC_PERSISTENT_RAM_H_ */

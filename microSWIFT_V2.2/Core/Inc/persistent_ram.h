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

// @formatter:off
typedef struct
{
  sbd_message_type_52       payload;
  bool                      valid;
} Iridium_Message_Storage_Element_t;

typedef struct
{
  sbd_message_type_53       payload;
  bool                      valid[TURBIDITY_MSGS_PER_SBD];
} Turbidity_Message_Storage_Element_t;

typedef struct
{
  sbd_message_type_54       payload;
  bool                      valid[LIGHT_MSGS_PER_SBD];
} Light_Message_Storage_Element_t;

#define MAX_NUM_NON_WAVES_MSGS_STORED   (65536U / ((sizeof(Iridium_Message_Storage_Element_t) * 5U) \
                                         + sizeof(Turbidity_Message_Storage_Element_t) \
                                         + sizeof(Light_Message_Storage_Element_t))) // Max possible based on 64K SRAM2 size

#define MAX_NUM_WAVES_MSGS_STORED       (MAX_NUM_NON_WAVES_MSGS_STORED * 5U)
#define MAX_NUM_TURBIDITY_MSGS_STORED   (MAX_NUM_NON_WAVES_MSGS_STORED * TURBIDITY_MSGS_PER_SBD)
#define MAX_NUM_LIGHT_MSGS_STORED       (MAX_NUM_NON_WAVES_MSGS_STORED * LIGHT_MSGS_PER_SBD)
typedef struct
{
  Iridium_Message_Storage_Element_t     msg_queue[MAX_NUM_WAVES_MSGS_STORED];
  uint32_t                              num_telemetry_msgs_enqueued;
} Waves_Message_Storage;

typedef struct
{
  Turbidity_Message_Storage_Element_t   msg_queue[MAX_NUM_NON_WAVES_MSGS_STORED];
  uint32_t                              num_msg_elements_enqueued;
} Turbidity_Message_Storage;

typedef struct
{
  Light_Message_Storage_Element_t       msg_queue[MAX_NUM_NON_WAVES_MSGS_STORED];
  uint32_t                              num_msg_elements_enqueued;
} Light_Message_Storage;

// All of the things we need to retain in standby mode
typedef struct
{
  uint64_t                  magic_number;
  int32_t                   sample_window_counter;
  Waves_Message_Storage     waves_storage;
  Turbidity_Message_Storage turbidity_storage;
  Light_Message_Storage     light_storage;
} Persistent_Storage;

typedef enum
{
  WAVES_TELEMETRY       = 0,
  TURBIDITY_TELEMETRY   = 1,
  LIGHT_TELEMETRY       = 2
} telemetry_type_t;

void                    persistent_ram_init ( void );
void                    persistent_ram_deinit ( void );
void                    persistent_ram_increment_sample_window_counter ( void );
int32_t                 persistent_ram_get_sample_window_counter ( void );
void                    persistent_ram_save_message ( telemetry_type_t msg_type, uint8_t* msg );
uint8_t*                persistent_ram_get_prioritized_unsent_message ( telemetry_type_t msg_type );
void                    persistent_ram_delete_message_element ( telemetry_type_t msg_type, uint8_t *msg_ptr );
// @formatter:on

#endif /* INC_PERSISTENT_RAM_H_ */

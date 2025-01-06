/*
 * file_system_server.c
 *
 *  Created on: Nov 21, 2024
 *      Author: philbush
 */

#include "file_system_server.h"
#include "string.h"

static file_system_server file_server_self;

void file_system_server_init ( TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags,
                               microSWIFT_configuration *global_config )
{
  file_server_self.request_queue = request_queue;
  file_server_self.complete_flags = complete_flags;
  file_server_self.global_config = global_config;
}

uSWIFT_return_code_t file_system_server_save_log_line ( char *log_line, ULONG complete_flag )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  file_system_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SAVE_LOG_LINE;
  queue_msg.size = strlen (log_line);
  queue_msg.object_pointer = (void*) log_line;
  queue_msg.complete_flag = complete_flag;
  queue_msg.return_code = &ret;

  if ( tx_queue_send (file_server_self.request_queue, &queue_msg,
  FILE_SYSTEM_QUEUE_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (file_server_self.complete_flags, complete_flag, TX_OR_CLEAR,
                           &event_flags, FILE_SYSTEM_FLAG_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_TIMEOUT;
  }

  return ret;
}

uSWIFT_return_code_t file_system_server_save_gnss_raw ( GNSS *gnss, ULONG complete_flag )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  file_system_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SAVE_GNSS_RAW;
  queue_msg.size = sizeof(float) * file_server_self.global_config->samples_per_window * 3;
  queue_msg.object_pointer = (void*) gnss;
  queue_msg.complete_flag = complete_flag;
  queue_msg.return_code = &ret;

  if ( tx_queue_send (file_server_self.request_queue, &queue_msg,
  FILE_SYSTEM_QUEUE_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (file_server_self.complete_flags, complete_flag, TX_OR_CLEAR,
                           &event_flags, FILE_SYSTEM_FLAG_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_TIMEOUT;
  }

  return ret;
}

uSWIFT_return_code_t file_system_server_save_gnss_track ( GNSS *gnss, ULONG complete_flag )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  file_system_request_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.request = SAVE_GNSS_BREADCRUMB_TRACK;
  queue_msg.size = sizeof(gnss_track_point) * (gnss->breadcrumb_index + 1);
  queue_msg.object_pointer = (void*) gnss;
  queue_msg.complete_flag = complete_flag;
  queue_msg.return_code = &ret;

  if ( tx_queue_send (file_server_self.request_queue, &queue_msg,
  FILE_SYSTEM_QUEUE_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (file_server_self.complete_flags, complete_flag, TX_OR_CLEAR,
                           &event_flags, FILE_SYSTEM_FLAG_MAX_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_TIMEOUT;
  }

  return ret;
}

uSWIFT_return_code_t file_system_server_save_temperature_raw ( Temperature *temp,
                                                               ULONG complete_flag );
uSWIFT_return_code_t file_system_server_save_ct_raw ( CT *ct, ULONG complete_flag );
uSWIFT_return_code_t file_system_server_save_light_raw ( Light_Sensor *light, ULONG complete_flag );
uSWIFT_return_code_t file_system_server_save_turbidity_raw ( Turbidity_Sensor *obs,
                                                             ULONG complete_flag );
uSWIFT_return_code_t file_system_server_close_out_files ( ULONG complete_flag );

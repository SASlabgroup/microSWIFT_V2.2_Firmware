/*
 * file_system_server.h
 *
 *  Created on: Nov 21, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_FILE_SYSTEM_SERVER_H_
#define COMPONENTS_INC_FILE_SYSTEM_SERVER_H_

#include "stddef.h"
#include "tx_api.h"
#include "fx_api.h"
#include "microSWIFT_return_codes.h"
#include "gnss.h"
#include "temp_sensor.h"
#include "ct_sensor.h"
#include "light_sensor.h"
#include "turbidity_sensor.h"

// @formatter:off

#define FILE_SYSTEM_QUEUE_LENGTH 16U
#define FILE_SYSTEM_QUEUE_MAX_WAIT_TICKS 2U
#define FILE_SYSTEM_FLAG_MAX_WAIT_TICKS 50U

// Define request types
typedef enum
{
  SAVE_LOG_LINE                 = 0,
  SAVE_GNSS_RAW                 = 1,
  SAVE_GNSS_BREADCRUMB_TRACK    = 2,
  SAVE_TEMPERATURE_RAW          = 3,
  SAVE_CT_RAW                   = 4,
  SAVE_LIGHT_RAW                = 5,
  SAVE_TURBIDITY_RAW            = 6,
  CLOSE_OUT_FILES               = 7
} file_system_request_t;

typedef struct
{
  int32_t               request; // use type file_system_request_t
  uint32_t              size;
  void                  *object_pointer;
  ULONG                 complete_flag;
  uSWIFT_return_code_t  *return_code;
} file_system_request_message;

typedef struct
{
  TX_QUEUE                  *request_queue;
  TX_EVENT_FLAGS_GROUP      *complete_flags;
  microSWIFT_configuration  *global_config;
} file_system_server;

void                    file_system_server_init ( TX_QUEUE *request_queue, TX_EVENT_FLAGS_GROUP *complete_flags,
                                                  microSWIFT_configuration *global_config );
uSWIFT_return_code_t    file_system_server_save_log_line ( char *log_line, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_gnss_raw ( GNSS *gnss, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_gnss_track ( GNSS *gnss, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_temperature_raw ( Temperature *temp, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_ct_raw ( CT *ct, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_light_raw ( Light_Sensor *light, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_save_turbidity_raw ( Turbidity_Sensor *obs, ULONG complete_flag );
uSWIFT_return_code_t    file_system_server_close_out_files ( ULONG complete_flag );



// @formatter:on
#endif /* COMPONENTS_INC_FILE_SYSTEM_SERVER_H_ */

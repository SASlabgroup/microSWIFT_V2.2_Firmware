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
#include "microSWIFT_return_codes.h"

// @formatter:off

#define FILE_SYSTEM_QUEUE_LENGTH 16U
#define FILE_SYSTEM_QUEUE_MAX_WAIT_TICKS 2U
#define FILE_SYSTEM_FLAG_MAX_WAIT_TICKS 50U

// Define request types
typedef enum
{
  SAVE_LOG_LINE             = 0,
  SAVE_GNSS_RAW             = 1,
  SAVE_TEMPERATURE_RAW      = 2,
  SAVE_CT_RAW               = 3,
  SAVE_LIGHT_RAW            = 4,
  SAVE_TURBIDITY_RAW        = 5,
  SAVE_WAVES_RESULTS        = 6,
  SAVE_WAVES_TELEMETRY      = 7,
  SAVE_LIGHT_TELEMETRY      = 8,
  SAVE_TURBIDITY_TELEMETRY  = 9,
  CLOSE_OUT_FILES           = 10
} file_system_request_t;



// @formatter:on
#endif /* COMPONENTS_INC_FILE_SYSTEM_SERVER_H_ */

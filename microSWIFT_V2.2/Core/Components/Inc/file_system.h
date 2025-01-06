/*
 * file_system.h
 *
 *  Created on: Nov 21, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_FILE_SYSTEM_H_
#define COMPONENTS_INC_FILE_SYSTEM_H_

#include "fx_api.h"
#include "microSWIFT_return_codes.h"
#include "gnss.h"
#include "temp_sensor.h"
#include "ct_sensor.h"
#include "light_sensor.h"
#include "turbidity_sensor.h"
#include "gpio.h"

// @formatter:off
#define FILE_SYSTEM_MAX_FILES (7U)

typedef enum
{
  LOG_FILE              = 0,
  GNSS_VELOCITIES       = 1,
  GNSS_BREADCRUMB_TRACK = 2,
  TEMPERATURE_FILE      = 3,
  CT_FILE               = 4,
  LIGHT_FILE            = 5,
  TURBIDITY_FILE        = 6
} file_index_t;

typedef struct
{
  FX_MEDIA                  *sd_card;
  FX_FILE                   files[FILE_SYSTEM_MAX_FILES];
  uint32_t                  *media_sector_cache;

  microSWIFT_configuration  *global_config;

  gpio_pin_struct           fet_pin;

  uSWIFT_return_code_t      (*initialize_card) ( void );
  uSWIFT_return_code_t      (*close_out_files) ( void );
  uSWIFT_return_code_t      (*save_log_line) ( char* line, uint32_t len );
  uSWIFT_return_code_t      (*save_gnss_velocities) ( GNSS *gnss );
  uSWIFT_return_code_t      (*save_gnss_breadcrumb_track) ( GNSS *gnss );
  uSWIFT_return_code_t      (*save_temperature_raw) ( Temperature *temp );
  uSWIFT_return_code_t      (*save_ct_raw) ( CT *ct );
  uSWIFT_return_code_t      (*save_light_raw) ( Light_Sensor *light );
  uSWIFT_return_code_t      (*save_turbidity_raw) ( Turbidity_Sensor *obs );
} File_System_SD_Card;

void file_system_init ( File_System_SD_Card *file_sytem, uint32_t *media_sector_cache,
                        FX_MEDIA *sd_card, microSWIFT_configuration *global_config );
void file_system_deinit ( void );


// @formatter:on
#endif /* COMPONENTS_INC_FILE_SYSTEM_H_ */

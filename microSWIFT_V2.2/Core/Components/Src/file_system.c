/*
 * file_system.c
 *
 *  Created on: Nov 21, 2024
 *      Author: philbush
 */

#include "file_system.h"
#include "fx_stm32_sd_driver.h"
#include "threadx_support.h"

static uSWIFT_return_code_t _file_system_initialize_card ( void );
static uSWIFT_return_code_t _file_system_close_out_files ( void );
static uSWIFT_return_code_t _file_system_save_log_line ( char *line, uint32_t len );
static uSWIFT_return_code_t _file_system_save_gnss_velocities ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_gnss_breadcrumb_track ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_temperature_raw ( Temperature *temp );
static uSWIFT_return_code_t _file_system_save_ct_raw ( CT *ct );
static uSWIFT_return_code_t _file_system_save_light_raw ( Light_Sensor *light );
static uSWIFT_return_code_t _file_system_save_turbidity_raw ( Turbidity_Sensor *obs );

static File_System_SD_Card *file_sys_self;

void file_system_init ( File_System_SD_Card *file_sytem, uint32_t *media_sector_cache,
                        FX_MEDIA *sd_card, microSWIFT_configuration *global_config )
{
  file_sys_self = file_system;

  file_sys_self->sd_card = sd_card;
  file_sys_self->media_sector_cache = media_sector_cache;
  file_sys_self->global_config = global_config;
}

void file_system_deinit ( void )
{
  return;
}

static uSWIFT_return_code_t _file_system_initialize_card ( void )
{
  UINT sd_status;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  // Initialize the SDMMC interface
  if ( !sdmmc2_init () )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  sd_status = fx_media_open(file_sys_self->sd_card, "microSWIFT", fx_stm32_sd_driver,
                            (VOID *)FX_NULL, (VOID *) file_sys_self->media_sector_cache,
                            FX_STM32_SD_DEFAULT_SECTOR_SIZE);

  if ( sd_status != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
  }

done:
  return ret;
}

static uSWIFT_return_code_t _file_system_close_out_files ( void );
static uSWIFT_return_code_t _file_system_save_log_line ( char *line, uint32_t len );
static uSWIFT_return_code_t _file_system_save_gnss_velocities ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_gnss_breadcrumb_track ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_temperature_raw ( Temperature *temp );
static uSWIFT_return_code_t _file_system_save_ct_raw ( CT *ct );
static uSWIFT_return_code_t _file_system_save_light_raw ( Light_Sensor *light );
static uSWIFT_return_code_t _file_system_save_turbidity_raw ( Turbidity_Sensor *obs );

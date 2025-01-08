/*
 * file_system.c
 *
 *  Created on: Nov 21, 2024
 *      Author: philbush
 */

#include "file_system.h"
#include "fx_stm32_sd_driver.h"
#include "threadx_support.h"
#include "sdmmc.h"
#include "persistent_ram.h"
#include "stdio.h"
#include "gpio.h"

#define GNSS_LINE_BUF_SIZE (256U)

// Struct functions
static uSWIFT_return_code_t _file_system_initialize_card ( void );
static uSWIFT_return_code_t _file_system_save_log_line ( char *line, uint32_t len );
static uSWIFT_return_code_t _file_system_save_gnss_velocities ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_gnss_breadcrumb_track ( GNSS *gnss );
static uSWIFT_return_code_t _file_system_save_temperature_raw ( Temperature *temp );
static uSWIFT_return_code_t _file_system_save_ct_raw ( CT *ct );
static uSWIFT_return_code_t _file_system_save_light_raw ( Light_Sensor *light );
static uSWIFT_return_code_t _file_system_save_turbidity_raw ( Turbidity_Sensor *obs );

// Helper functions
static bool __open_sd_card ( void );
static bool __close_sd_card ( void );

// Internal object pointer
static File_System_SD_Card *file_sys_self;

// File write buffer for complex writes
static char write_buf[COMPLEX_FILE_WRITE_BUF_LEN];

// Constant char string defs
static char *gnss_csv_header = "Time, V North, V East, V Down\n";

void file_system_init ( File_System_SD_Card *file_system, uint32_t *media_sector_cache,
                        FX_MEDIA *sd_card, microSWIFT_configuration *global_config )
{
  file_sys_self = file_system;

  file_sys_self->sd_card = sd_card;
  file_sys_self->media_sector_cache = media_sector_cache;
  file_sys_self->global_config = global_config;

#warning "Add SD card fet pin here."

  file_sys_self->timer_timeout = false;

  file_sys_self->sample_window_counter = persistent_ram_get_sample_window_counter ();

  // Fill in the file names
  snprintf (&(file_sys_self->file_names[LOG_FILE][0]), FILE_MAX_NAME_LEN, "Logs/Log_%lu.txt",
            file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[GNSS_VELOCITIES][0]), FILE_MAX_NAME_LEN,
            "GNSS/Velocity%lu.csv", file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[GNSS_BREADCRUMB_TRACK][0]), FILE_MAX_NAME_LEN,
            "Tracks/Track_%lu.kml", file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[TEMPERATURE_FILE][0]), FILE_MAX_NAME_LEN,
            "Temperature/Temp_%lu.csv", file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[CT_FILE][0]), FILE_MAX_NAME_LEN, "CT/CT_%lu.csv",
            file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[LIGHT_FILE][0]), FILE_MAX_NAME_LEN, "Light/Light_%lu.csv",
            file_sys_self->sample_window_counter);
  snprintf (&(file_sys_self->file_names[TURBIDITY_FILE][0]), FILE_MAX_NAME_LEN,
            "Turbidity/Turbidity_%lu.csv", file_sys_self->sample_window_counter);

  file_sys_self->initialize_card = _file_system_initialize_card;
  file_sys_self->save_log_line = _file_system_save_log_line;
  file_sys_self->save_gnss_velocities = _file_system_save_gnss_velocities;
  file_sys_self->save_gnss_breadcrumb_track = _file_system_save_gnss_breadcrumb_track;
  file_sys_self->save_temperature_raw = _file_system_save_temperature_raw;
  file_sys_self->save_ct_raw = _file_system_save_ct_raw;
  file_sys_self->save_light_raw = _file_system_save_light_raw;
  file_sys_self->save_turbidity_raw = _file_system_save_turbidity_raw;
}

void file_system_deinit ( void )
{
#warning "Close out the media, shut down power, deinit sdmmc interface."
  return;
}

void file_system_timer_expired ( ULONG expiration_input )
{
  file_sys_self->timer_timeout = true;
}

bool file_system_get_timeout_status ( void )
{
  return file_sys_self->timer_timeout;
}

static uSWIFT_return_code_t _file_system_initialize_card ( void )
{
  UINT fx_ret;
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  // Initialize the SDMMC interface
  if ( !sdmmc2_init () )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  if ( !__open_sd_card () )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  // Create the directories in the first sample window
  if ( file_sys_self->sample_window_counter == 0 )
  {
    fx_ret = fx_directory_create (file_sys_self->sd_card, "Log");
    if ( fx_ret != FX_SUCCESS )
    {
      ret = uSWIFT_IO_ERROR;
      goto done;
    }

    fx_ret = fx_directory_create (file_sys_self->sd_card, "GNSS");
    if ( fx_ret != FX_SUCCESS )
    {
      ret = uSWIFT_IO_ERROR;
      goto done;
    }

    fx_ret = fx_directory_create (file_sys_self->sd_card, "Tracks");
    if ( fx_ret != FX_SUCCESS )
    {
      ret = uSWIFT_IO_ERROR;
      goto done;
    }

    if ( file_sys_self->global_config->temperature_enabled )
    {
      fx_ret = fx_directory_create (file_sys_self->sd_card, "Temperature");
      if ( fx_ret != FX_SUCCESS )
      {
        ret = uSWIFT_IO_ERROR;
        goto done;
      }
    }

    if ( file_sys_self->global_config->ct_enabled )
    {
      fx_ret = fx_directory_create (file_sys_self->sd_card, "CT");
      if ( fx_ret != FX_SUCCESS )
      {
        ret = uSWIFT_IO_ERROR;
        goto done;
      }
    }

    if ( file_sys_self->global_config->light_enabled )
    {
      fx_ret = fx_directory_create (file_sys_self->sd_card, "Light");
      if ( fx_ret != FX_SUCCESS )
      {
        ret = uSWIFT_IO_ERROR;
        goto done;
      }
    }

    if ( file_sys_self->global_config->turbidity_enabled )
    {
      fx_ret = fx_directory_create (file_sys_self->sd_card, "Turbidity");
      if ( fx_ret != FX_SUCCESS )
      {
        ret = uSWIFT_IO_ERROR;
        goto done;
      }
    }
  }

done:
  __close_sd_card ();
  return ret;
}

static uSWIFT_return_code_t _file_system_save_log_line ( char *line, uint32_t len )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  UINT fx_ret;
  static bool first_time = true, file_creation_failed = false;
  static uint32_t seek_index = 0;

  if ( !__open_sd_card () )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  if ( first_time )
  {
    // Create the file
    fx_ret = fx_file_create (file_sys_self->sd_card, &(file_sys_self->file_names[LOG_FILE][0]));
    if ( fx_ret != FX_SUCCESS )
    {
      file_creation_failed = true;
      ret = uSWIFT_IO_ERROR;
      goto done;
    }

    first_time = false;
  }

  if ( file_creation_failed )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_file_open(file_sys_self->sd_card, &file_sys_self->files[LOG_FILE],
                        &(file_sys_self->file_names[LOG_FILE][0]), FX_OPEN_FOR_WRITE);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_file_seek (&file_sys_self->files[LOG_FILE], seek_index);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_file_write (&file_sys_self->files[LOG_FILE], line, len);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  seek_index += len;

  fx_ret = fx_file_close (&file_sys_self->files[LOG_FILE]);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_media_flush (file_sys_self->sd_card);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

done:
  __close_sd_card ();
  return ret;
}

static uSWIFT_return_code_t _file_system_save_gnss_velocities ( GNSS *gnss )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  UINT fx_ret;
  char line[GNSS_LINE_BUF_SIZE];
  struct tm time;
  time_t timestamp = gnss->sample_window_start_time;
  size_t str_index, size_req;
  uint32_t velocity_index = 0, time_counter = 0;

  if ( !__open_sd_card () )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_file_open(file_sys_self->sd_card, &file_sys_self->files[GNSS_VELOCITIES],
                        &(file_sys_self->file_names[GNSS_VELOCITIES][0]), FX_OPEN_FOR_WRITE);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_file_seek (&file_sys_self->files[GNSS_VELOCITIES], 0);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  // Write the CSV file header
  fx_ret = fx_file_write (&file_sys_self->files[GNSS_VELOCITIES], gnss_csv_header,
                          strlen (gnss_csv_header));
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  // Write the CSV file files
  while ( velocity_index < gnss->total_samples )
  {
    // Update the timestamp second when appropriate
    if ( ((time_counter++ % file_sys_self->global_config->gnss_sampling_rate) == 0)
         && (velocity_index != 0) )
    {
      time = *gmtime (&timestamp);
      timestamp++;
    }

    // Write the timstamp string
    str_index = strftime (&(line[0]), GNSS_LINE_BUF_SIZE, "%X %x,", &time);

    // Write the velocities
    size_req = snprintf (&(line[str_index]), GNSS_LINE_BUF_SIZE - str_index, "%.3f,%.3f,%.3f",
                         gnss->GNSS_N_Array[velocity_index], gnss->GNSS_E_Array[velocity_index],
                         gnss->GNSS_D_Array[velocity_index]);
    // Make sure we got everything
    if ( size_req > (GNSS_LINE_BUF_SIZE - str_index) )
    {
      ret = uSWIFT_MEMORY_BUFFER_ERROR;
      goto done;
    }
    // Write it!
    fx_ret = fx_file_write (&file_sys_self->files[GNSS_VELOCITIES], &(line[0]), strlen (line));
    if ( fx_ret != FX_SUCCESS )
    {
      ret = uSWIFT_IO_ERROR;
      goto done;
    }

    velocity_index++;
  }

  fx_ret = fx_file_close (&file_sys_self->files[GNSS_VELOCITIES]);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

  fx_ret = fx_media_flush (file_sys_self->sd_card);
  if ( fx_ret != FX_SUCCESS )
  {
    ret = uSWIFT_IO_ERROR;
    goto done;
  }

done:
  __close_sd_card ();
  return ret;
}

static uSWIFT_return_code_t _file_system_save_gnss_breadcrumb_track ( GNSS *gnss )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _file_system_save_temperature_raw ( Temperature *temp )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _file_system_save_ct_raw ( CT *ct )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _file_system_save_light_raw ( Light_Sensor *light )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static uSWIFT_return_code_t _file_system_save_turbidity_raw ( Turbidity_Sensor *obs )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  return ret;
}

static bool __open_sd_card ( void )
{
  UINT fx_ret;

  gpio_write_pin (file_sys_self->fet_pin, GPIO_PIN_SET);

  tx_thread_sleep (1);

  fx_ret = fx_media_open(file_sys_self->sd_card, "microSWIFT", fx_stm32_sd_driver, (VOID *)FX_NULL,
                         (VOID *) file_sys_self->media_sector_cache,
                         FX_STM32_SD_DEFAULT_SECTOR_SIZE);

  return (fx_ret == FX_SUCCESS);
}

static bool __close_sd_card ( void )
{
  UINT fx_ret;

  fx_ret = fx_media_close (file_sys_self->sd_card);

  gpio_write_pin (file_sys_self->fet_pin, GPIO_PIN_SET);

  return (fx_ret == FX_SUCCESS);
}


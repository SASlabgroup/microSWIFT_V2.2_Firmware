/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_filex.c
 * @author  MCD Application Team
 * @brief   FileX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_filex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_threadx.h"
#include "sdmmc.h"
#include "logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* Main thread stack size */
#define FX_APP_THREAD_STACK_SIZE         8192
/* Main thread priority */
#define FX_APP_THREAD_PRIO               7
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Main thread global data structures.  */
TX_THREAD fx_app_thread;

/* Buffer for FileX FX_MEDIA sector cache. */
ALIGN_32BYTES(uint32_t fx_sd_media_memory[FX_STM32_SD_DEFAULT_SECTOR_SIZE / sizeof(uint32_t)]);
/* Define FileX global data structures.  */
FX_MEDIA sdio_disk;

/* USER CODE BEGIN PV */
FX_FILE fx_file;
HAL_SD_CardInfoTypeDef sd_info;
HAL_SD_CardInfoTypeDef *pCardInfoSD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* Main thread entry function.  */
void fx_app_thread_entry ( ULONG thread_input );

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
 * @brief  Application FileX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT MX_FileX_Init ( VOID *memory_ptr )
{
  UINT ret = FX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*) memory_ptr;
  VOID *pointer;

  /* USER CODE BEGIN MX_FileX_MEM_POOL */

  /* USER CODE END MX_FileX_MEM_POOL */

  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */

  /*Allocate memory for the main thread's stack*/
  ret = tx_byte_allocate (byte_pool, &pointer, FX_APP_THREAD_STACK_SIZE, TX_NO_WAIT);

  /* Check FX_APP_THREAD_STACK_SIZE allocation*/
  if ( ret != FX_SUCCESS )
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread.  */
  ret = tx_thread_create(&fx_app_thread, FX_APP_THREAD_NAME, fx_app_thread_entry, 0, pointer,
                         FX_APP_THREAD_STACK_SIZE, FX_APP_THREAD_PRIO, FX_APP_PREEMPTION_THRESHOLD,
                         FX_APP_THREAD_TIME_SLICE, FX_APP_THREAD_AUTO_START);

  /* Check main thread creation */
  if ( ret != FX_SUCCESS )
  {
    return TX_THREAD_ERROR;
  }

  /* USER CODE BEGIN MX_FileX_Init */

  /* USER CODE END MX_FileX_Init */

  /* Initialize FileX.  */
  fx_system_initialize ();

  /* USER CODE BEGIN MX_FileX_Init 1*/

  /* USER CODE END MX_FileX_Init 1*/

  return ret;
}

/**
 * @brief  Main thread entry.
 * @param thread_input: ULONG user argument used by the thread entry
 * @retval none
 */
void fx_app_thread_entry ( ULONG thread_input )
{

  UINT sd_status = FX_SUCCESS;

  /* USER CODE BEGIN fx_app_thread_entry 0*/
  TX_THREAD *this_thread = tx_thread_identify ();
  CHAR data[] = "This is FileX working on STM32";
  ULONG bytes_read;
  CHAR read_buffer[32];

  tx_thread_sleep (10);

  if ( !sdmmc1_init () )
  {
    filex_error_out (this_thread, "SDMMC1 failed to initialize.");
  }

  LOG("SD card peripheral initialization successful.");

  if ( HAL_SD_GetCardInfo (&hsd1, &sd_info) != HAL_OK )
  {
    filex_error_out (this_thread, "SDMMC1 failed to initialize.");
  }

  pCardInfoSD = &sd_info;
  if ( is_first_sample_window () )
  {
    sd_status = fx_media_format (&sdio_disk,                          // SD_Disk pointer
        fx_stm32_sd_driver,                  // Driver entry
        (VOID*) FX_NULL,                     // Device info pointer
        (UCHAR*) fx_sd_media_memory,        // Media buffer pointer
        sizeof(fx_sd_media_memory),          // Media buffer size
        FX_SD_VOLUME_NAME,                   // Volume Name
        FX_SD_NUMBER_OF_FATS,                // Number of FATs
        32,                                  // Directory Entries
        FX_SD_HIDDEN_SECTORS,                // Hidden sectors
        pCardInfoSD->BlockNbr,                 // Total sectors
        FX_STM32_SD_DEFAULT_SECTOR_SIZE,     // Sector size
        8,                                   // Sectors per cluster
        1,                                   // Heads
        1);                                  // Sectors per track

    /* Check the format sd_status */
    if ( sd_status != FX_SUCCESS )
    {
      filex_error_out (this_thread, "SD card format failed.");
    }

    LOG("SD card formatted.");
  }

  /* Open the SD disk driver */
  sd_status = fx_media_open(&sdio_disk, FX_SD_VOLUME_NAME, fx_stm32_sd_driver, (VOID *)FX_NULL,
                            (VOID *) fx_sd_media_memory, sizeof(fx_sd_media_memory));

  /* Check the media open sd_status */
  if ( sd_status != FX_SUCCESS )
  {
    /* USER CODE BEGIN SD DRIVER get info error */
    filex_error_out (this_thread, "FileX failed to initialize.");
    /* USER CODE END SD DRIVER get info error */
  }

  LOG("File system initialization successful.");

  /* USER CODE BEGIN fx_app_thread_entry 1*/
  /* Create a file called STM32.TXT in the root directory.  */
  sd_status = fx_file_create (&sdio_disk, "STM32.TXT");

  /* Check the create status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Check for an already created status. This is expected on the
     second pass of this loop!  */
    if ( sd_status != FX_ALREADY_CREATED )
    {
      /* Create error, call error handler.  */
      Error_Handler ();
    }
  }

  /* Open the test file.  */
  sd_status = fx_file_open(&sdio_disk, &fx_file, "STM32.TXT", FX_OPEN_FOR_WRITE);

  /* Check the file open status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error opening file, call error handler.  */
    Error_Handler ();
  }

  /* Seek to the beginning of the test file.  */
  sd_status = fx_file_seek (&fx_file, 0);

  /* Check the file seek status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error performing file seek, call error handler.  */
    Error_Handler ();
  }

  /* Write a string to the test file.  */
  sd_status = fx_file_write (&fx_file, data, sizeof(data));

  /* Check the file write status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error writing to a file, call error handler.  */
    Error_Handler ();
  }

  /* Close the test file.  */
  sd_status = fx_file_close (&fx_file);

  /* Check the file close status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error closing the file, call error handler.  */
    Error_Handler ();
  }

  sd_status = fx_media_flush (&sdio_disk);

  /* Check the media flush  status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error closing the file, call error handler.  */
    Error_Handler ();
  }

  /* Open the test file.  */
  sd_status = fx_file_open(&sdio_disk, &fx_file, "STM32.TXT", FX_OPEN_FOR_READ);

  /* Check the file open status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error opening file, call error handler.  */
    Error_Handler ();
  }

  /* Seek to the beginning of the test file.  */
  sd_status = fx_file_seek (&fx_file, 0);

  /* Check the file seek status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error performing file seek, call error handler.  */
    Error_Handler ();
  }

  /* Read the first 28 bytes of the test file.  */
  sd_status = fx_file_read (&fx_file, read_buffer, sizeof(data), &bytes_read);

  /* Check the file read status.  */
  if ( (sd_status != FX_SUCCESS) || (bytes_read != sizeof(data)) )
  {
    /* Error reading file, call error handler.  */
    Error_Handler ();
  }

  /* Close the test file.  */
  sd_status = fx_file_close (&fx_file);

  /* Check the file close status. */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error closing the file, call error handler. */
    Error_Handler ();
  }

  /* Close the media.  */
  sd_status = fx_media_close (&sdio_disk);

  /* Check the media close status.  */
  if ( sd_status != FX_SUCCESS )
  {
    /* Error closing the media, call error handler.  */
    Error_Handler ();
  }

  LOG("File system complete.");
  /* USER CODE END fx_app_thread_entry 1*/
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

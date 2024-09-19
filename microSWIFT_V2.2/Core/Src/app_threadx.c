/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 ******************************************************************************
 *
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stddef.h"
#include <math.h>
#include "stm32u5xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "ext_rtc.h"
#include "gnss.h"
#include "battery.h"
#include "ct_sensor.h"
#include "rf_switch.h"
#include "iridium.h"
#include "temp_sensor.h"
#include "NEDWaves/mem_replacements.h"
#include "configuration.h"
#include "linked_list.h"
#include "testing_hooks.h"
#include "adc.h"
#include "logger.h"
#include "threadx_support.h"
#include "watchdog.h"
#include "controller.h"
#include "persistent_ram.h"
#include "sbd.h"

// Waves files
#include "NEDWaves/NEDwaves_memlight.h"
#include "NEDWaves/NEDwaves_memlight_emxAPI.h"
#include "NEDWaves/NEDwaves_memlight_terminate.h"
#include "NEDWaves/NEDwaves_memlight_types.h"
#include "NEDWaves/rt_nonfinite.h"
#include "NEDWaves/rtwhalf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum thread_priorities
{
  HIGHEST_PRIORITY = 1,
  VERY_HIGH_PRIORITY = 2,
  HIGH_PRIORITY = 3,
  MID_PRIORITY = 4,
  LOW_PRIORITY = 5,
  VERY_LOW_PRIORITY = 6,
  LOWEST_PRIORITY = 7
};

enum stack_sizes
{
  XXL_STACK = 16384,
  XL_STACK = 8192,
  L_STACK = 6144,
  M_STACK = 4096,
  S_STACK = 2048,
  XS_STACK = 1024,
  XXS_STACK = 512
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// The configuration struct
microSWIFT_configuration configuration;
// The SBD message we'll assemble
sbd_message_type_52 sbd_message;
// Handles for all the STM32 peripherals
Device_Handles device_handles;
// The primary byte pool from which all memory is allocated from
TX_BYTE_POOL *byte_pool;
// Our threads
TX_THREAD control_thread;
TX_THREAD rtc_thread;
TX_THREAD logger_thread;
TX_THREAD gnss_thread;
TX_THREAD ct_thread;
TX_THREAD temperature_thread;
TX_THREAD light_thread;
TX_THREAD turbidity_thread;
TX_THREAD accelerometer_thread;
TX_THREAD waves_thread;
TX_THREAD iridium_thread;
TX_THREAD expansion_thread_1;
TX_THREAD expansion_thread_2;
TX_THREAD expansion_thread_3;
// @formatter:off
Thread_Handles thread_handles =
  {
    &control_thread,
    &rtc_thread,
    &logger_thread,
    &gnss_thread,
    &ct_thread,
    &temperature_thread,
    &light_thread,
    &turbidity_thread,
    &accelerometer_thread,
    &waves_thread,
    &iridium_thread,
    &expansion_thread_1,
    &expansion_thread_2,
    &expansion_thread_3
  };
// @formatter:on
// Used to track initialization status of threads and components
TX_EVENT_FLAGS_GROUP initialization_flags;
// We'll use these flags to indicate a thread has completed execution
TX_EVENT_FLAGS_GROUP complete_flags;
// These flags are used to indicate an interrupt event has occurred
TX_EVENT_FLAGS_GROUP irq_flags;
// Flags for errors
TX_EVENT_FLAGS_GROUP error_flags;
// RTC complete flags
TX_EVENT_FLAGS_GROUP rtc_complete_flags;
// Flags for which thread is checkin in with watchdog
TX_EVENT_FLAGS_GROUP watchdog_check_in_flags;
// Expansion event flags
TX_EVENT_FLAGS_GROUP expansion_flags_1;
TX_EVENT_FLAGS_GROUP expansion_flags_2;
TX_EVENT_FLAGS_GROUP expansion_flags_3;
// Timers for threads
TX_TIMER control_timer;
TX_TIMER gnss_timer;
TX_TIMER ct_timer;
TX_TIMER temperature_timer;
TX_TIMER light_timer;
TX_TIMER turbidity_timer;
TX_TIMER accelerometer_timer;
TX_TIMER waves_timer;
TX_TIMER iridium_timer;
TX_TIMER expansion_timer_1;
TX_TIMER expansion_timer_2;
TX_TIMER expansion_timer_3;
// Comms buses semaphores
TX_SEMAPHORE ext_rtc_spi_sema;
TX_SEMAPHORE aux_spi_1_spi_sema;
TX_SEMAPHORE aux_spi_2_spi_sema;
TX_SEMAPHORE aux_i2c_1_sema;
TX_SEMAPHORE aux_i2c_2_sema;
TX_SEMAPHORE iridium_uart_sema;
TX_SEMAPHORE ct_uart_sema;
TX_SEMAPHORE aux_uart_1_sema;
TX_SEMAPHORE aux_uart_2_sema;
// Shared bus locks
TX_MUTEX core_i2c_mutex;
// Server/client message queue for RTC (including watchdog function)
TX_QUEUE rtc_messaging_queue;
TX_QUEUE logger_message_queue;
TX_QUEUE expansion_queue_1;
TX_QUEUE expansion_queue_2;

__ALIGN_BEGIN UCHAR waves_byte_pool_buffer[WAVES_MEM_POOL_SIZE] __ALIGN_END;
TX_BYTE_POOL waves_byte_pool;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

// Threads
static void rtc_thread_entry ( ULONG thread_input );
static void logger_thread_entry ( ULONG thread_input );
static void control_thread_entry ( ULONG thread_input );
static void gnss_thread_entry ( ULONG thread_input );
static void waves_thread_entry ( ULONG thread_input );
static void iridium_thread_entry ( ULONG thread_input );
static void ct_thread_entry ( ULONG thread_input );
static void temperature_thread_entry ( ULONG thread_input );
static void light_thread_entry ( ULONG thread_input );
static void turbidity_thread_entry ( ULONG thread_input );
static void accelerometer_thread_entry ( ULONG thread_input );
static void expansion_thread_1_entry ( ULONG thread_input );
static void expansion_thread_2_entry ( ULONG thread_input );
static void expansion_thread_3_entry ( ULONG thread_input );

/* USER CODE END PFP */

/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init ( VOID *memory_ptr )
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void) byte_pool;
  CHAR *pointer = TX_NULL;
  byte_pool = memory_ptr;

  /************************************************************************************************
   ************************************** Threads *************************************************
   ************************************************************************************************/
  //
  // Allocate stack for the control thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XL_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the control thread. HIGHEST priority level and no preemption possible
  ret = tx_thread_create(&control_thread, "control thread", control_thread_entry, 0, pointer,
                         XL_STACK, HIGHEST_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the rtc thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XS_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the rtc thread. VERY_HIGH priority level and no preemption possible
  ret = tx_thread_create(&rtc_thread, "rtc thread", rtc_thread_entry, 0, pointer, XS_STACK,
                         VERY_HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the logger thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, L_STACK,
  TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the logger thread. Low priority priority level and no preemption possible
  ret = tx_thread_create(&logger_thread, "logger thread", logger_thread_entry, 0, pointer, L_STACK,
                         LOW_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the gnss thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, L_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the gnss thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&gnss_thread, "gnss thread", gnss_thread_entry, 0, pointer, L_STACK,
                         MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the CT thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, S_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the CT thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&ct_thread, "ct thread", ct_thread_entry, 0, pointer, S_STACK,
                         MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the temperature thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XS_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the temperature thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&temperature_thread, "temperature thread", temperature_thread_entry, 0,
                         pointer, XS_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the light thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XS_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the light thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&light_thread, "light thread", light_thread_entry, 0, pointer, XS_STACK,
                         MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the turbidity thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XS_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the turbidity thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&turbidity_thread, "turbidity thread", turbidity_thread_entry, 0, pointer,
                         XS_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the accelerometer thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, L_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the accelerometer thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&accelerometer_thread, "accelerometer thread", accelerometer_thread_entry,
                         0, pointer, L_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the waves thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XL_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the waves thread. HIGH priority, no preemption-threshold
  ret = tx_thread_create(&waves_thread, "waves thread", waves_thread_entry, 0, pointer, XL_STACK,
                         HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Allocate stack for the Iridium thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the Iridium thread. HIGH priority, no preemption-threshold
  ret = tx_thread_create(&iridium_thread, "iridium thread", iridium_thread_entry, 0, pointer,
                         M_STACK, HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  // Create the expansion threads
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_thread_create(&expansion_thread_1, "expansion_thread_1", expansion_thread_1_entry, 0,
                         pointer, M_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_thread_create(&expansion_thread_2, "expansion_thread_2", expansion_thread_2_entry, 0,
                         pointer, M_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_thread_create(&expansion_thread_3, "expansion_thread_3", expansion_thread_3_entry, 0,
                         pointer, M_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ************************************** Event Flags *********************************************
   ************************************************************************************************/
  //
  // For tracking initialization of components and threads
  ret = tx_event_flags_create(&initialization_flags, "init flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Create the event flags we'll use for tracking thread completion
  ret = tx_event_flags_create(&complete_flags, "completion flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Create the event flags we'll use for tracking interrupt events
  ret = tx_event_flags_create(&irq_flags, "interrupt flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Create the error flags we'll use for tracking errors
  ret = tx_event_flags_create(&error_flags, "error flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Create the rtc complete flags we'll use for tracking rtc function completion
  ret = tx_event_flags_create(&rtc_complete_flags, "RTC complete flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  //
  // Create watchdog check in flags -- used to track which thread has checked in
  ret = tx_event_flags_create(&watchdog_check_in_flags, "watchdog check-in flags");
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   **************************************** Timers ************************************************
   ************************************************************************************************/
  ret = tx_timer_create(&control_timer, "Control thread timer", control_timer_expired, 0, 0, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&gnss_timer, "GNSS thread timer", gnss_timer_expired, 0, 0, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&ct_timer, "CT thread timer", ct_timer_expired, 0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&temperature_timer, "Temperature thread timer", temperature_timer_expired,
                        0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&light_timer, "Light thread timer", light_timer_expired, 0, 0, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&turbidity_timer, "Turbidity thread timer", turbidity_timer_expired, 0, 0,
                        0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&accelerometer_timer, "Accelerometer thread timer",
                        accelerometer_timer_expired, 0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&waves_timer, "Waves thread timer", waves_timer_expired, 0, 0, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&iridium_timer, "Iridium thread timer", iridium_timer_expired, 0, 0, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&expansion_timer_1, "Expansion thread 1 timer", expansion_timer_1_expired,
                        0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&expansion_timer_2, "Expansion thread 2 timer", expansion_timer_2_expired,
                        0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&expansion_timer_3, "Expansion thread 3 timer", expansion_timer_3_expired,
                        0, 0, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ************************************** Semaphores **********************************************
   ************************************************************************************************/
  //
  // Semaphores to identify comms bus DMA Tx/Rx completion
  ret = tx_semaphore_create(&ext_rtc_spi_sema, "RTC SPI sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_spi_1_spi_sema, "Aux SPI 1 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_spi_2_spi_sema, "Aux SPI 2 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_i2c_1_sema, "Aux I2C 1 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_i2c_2_sema, "Aux I2C 2 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&iridium_uart_sema, "Iridium UART sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&ct_uart_sema, "CT UART sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_uart_1_sema, "Aux UART 1 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_semaphore_create(&aux_uart_2_sema, "Aux UART 2 sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   **************************************** Mutexes ***********************************************
   ************************************************************************************************/
  //
  // Mutexes for shared communication busses
  ret = tx_mutex_create(&core_i2c_mutex, "Core I2C mutex", TX_NO_INHERIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ************************************** Message Queues ******************************************
   ************************************************************************************************/
  // Server message queue for RTC (including watchdog function) and UART Logger
  //
  // Allocate buffer space for the message queue
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, sizeof(logger_message) * LOG_QUEUE_LENGTH,
  TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&logger_message_queue, "logger msg queue",
                        sizeof(logger_message) / sizeof(uint32_t), pointer,
                        sizeof(logger_message) * LOG_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
                          sizeof(rtc_request_message) * RTC_QUEUE_LENGTH, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&logger_message_queue, "RTC msg queue",
                        sizeof(rtc_request_message) / sizeof(uint32_t), pointer,
                        sizeof(rtc_request_message) * RTC_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
  EXPANSION_QUEUE_MSG_SIZE * EXPANSION_QUEUE_LENGTH,
                          TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&expansion_queue_1, "Expansion queue 1",
                        EXPANSION_QUEUE_MSG_SIZE / sizeof(uint32_t), pointer,
                        EXPANSION_QUEUE_MSG_SIZE * EXPANSION_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
  EXPANSION_QUEUE_MSG_SIZE * EXPANSION_QUEUE_LENGTH,
                          TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&expansion_queue_2, "Expansion queue 2",
                        EXPANSION_QUEUE_MSG_SIZE / sizeof(uint32_t), pointer,
                        EXPANSION_QUEUE_MSG_SIZE * EXPANSION_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ***************************************** Misc init ********************************************
   ************************************************************************************************/
  device_handles.core_spi_handle = &hspi1;
  device_handles.core_i2c_handle = &hi2c1;
  device_handles.iridium_uart_handle = &huart4;
  device_handles.gnss_uart_handle = &huart1;
  device_handles.ct_uart_handle = &huart5;
  device_handles.ext_flash_handle = &hospi1;
  device_handles.battery_adc = &hadc1;
  device_handles.aux_spi_1_handle = &hspi2;
  device_handles.aux_spi_2_handle = &hspi3;
  device_handles.aux_i2c_1_handle = &hi2c2;
  device_handles.aux_i2c_2_handle = &hi2c3;
  device_handles.aux_uart_1_handle = &huart2;
  device_handles.aux_uart_2_handle = &huart3;
  device_handles.gnss_uart_tx_dma_handle = &handle_GPDMA1_Channel1;
  device_handles.gnss_uart_rx_dma_handle = &handle_GPDMA1_Channel0;
  device_handles.iridium_uart_tx_dma_handle = &handle_GPDMA1_Channel9;
  device_handles.iridium_uart_rx_dma_handle = &handle_GPDMA1_Channel8;
  device_handles.ct_uart_tx_dma_handle = &handle_GPDMA1_Channel11;
  device_handles.ct_uart_tx_dma_handle = &handle_GPDMA1_Channel10;
  device_handles.aux_uart_1_tx_dma_handle = &handle_GPDMA1_Channel3;
  device_handles.aux_uart_1_rx_dma_handle = &handle_GPDMA1_Channel2;
  device_handles.aux_uart_2_tx_dma_handle = &handle_GPDMA1_Channel5;
  device_handles.aux_uart_2_rx_dma_handle = &handle_GPDMA1_Channel4;

  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.windows_per_hour = SAMPLE_WINDOWS_PER_HOUR;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_max_acquisition_wait_time = GNSS_MAX_ACQUISITION_WAIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
  configuration.total_ct_samples = TOTAL_CT_SAMPLES;
  configuration.total_temp_samples = TOTAL_TEMPERATURE_SAMPLES;
  configuration.total_light_samples = TOTAL_LIGHT_SAMPLES;
  configuration.total_turbidity_samples = TOTAL_TURBIDITY_SAMPLES;
  configuration.gnss_high_performance_mode = GNSS_HIGH_PERFORMANCE_MODE_ENABLED;
  configuration.ct_enabled = CT_ENABLED;
  configuration.temperature_enabled = TEMPERATURE_ENABLED;
  configuration.light_enabled = LIGHT_SENSOR_ENABLED;
  configuration.turbidity_enabled = TURBIDITY_SENSOR_ENABLED;
  configuration.accelerometer_enabled = ACCELEROMETER_ENABLED;

  /* USER CODE END App_ThreadX_MEM_POOL */
  /* USER CODE BEGIN App_ThreadX_Init */
  //
  // Run tests if needed
  if ( tests.threadx_init_test != NULL )
  {
    tests.threadx_init_test (NULL);
  }
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
 * @brief  Function that implements the kernel's initialization.
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init ( void )
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter ();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/***************************************************************************************************
 ***************************************************************************************************
 *************************************    RTC Thread    ********************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  RTC thread. Manages all RTC operations.
 *
 * @note   This is a server style thread to manage shared hardware. Multiple threads require access
 *         to the RTC, so all operations happen in this thread. There are two methods of notifying
 *         this thread that there is work to do:
 *
 *         1) A watchdog semaphore -- if this semaphore ever has a count greater than 0, the
 *            RTC will refresh the watchdog.
 *
 *         2) For all other RTC requests, functions in ext_rtc_api.h are called which place a
 *            rtc_request_message on the queue, which contains all the required information for an
 *            RTC operation. The return code is written back via pointer provided in the queue
 *            message.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void rtc_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &rtc_thread;
  Ext_RTC rtc;
  rtc_return_code ret;
  UINT tx_ret;
  rtc_request_message req;

  ret = ext_rtc_init (&rtc, device_handles.core_spi_handle, &rtc_messaging_queue,
                      &rtc_complete_flags);

  // Initialize the watchdog
  ret |= rtc.config_watchdog (WATCHDOG_PERIOD);

  if ( ret != RTC_SUCCESS )
  {
    uart_logger_log_line ("RTC failed to initialize.");
    (void) tx_thread_suspend (this_thread);
  }

  (void) tx_event_flags_set (&initialization_flags, RTC_INIT_SUCCESS, TX_OR);
  uart_logger_log_line ("RTC initialization successful.");

  while ( 1 )
  {
    // See if we have any requests on the queue
    tx_ret = tx_queue_receive (&rtc_messaging_queue, &req, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      switch ( req.request )
      {
        case REFRESH_WATCHDOG:
          ret = rtc.refresh_watchdog ();
          break;

        case GET_TIME:
          ret = rtc.get_date_time (&req.input_output_struct->get_set_time.time_struct);
          break;

        case SET_TIME:

          ret = rtc.set_date_time (req.input_output_struct->get_set_time.time_struct);
          break;

        case SET_TIMESTAMP:
          ret = rtc.set_timestamp (req.input_output_struct->get_set_timestamp.which_timestamp);
          break;

        case GET_TIMESTAMP:
          ret = rtc.get_timestamp (req.input_output_struct->get_set_timestamp.which_timestamp,
                                   &req.input_output_struct->get_set_timestamp.timestamp);
          break;

        case SET_ALARM:
          ret = rtc.set_alarm (req.input_output_struct->set_alarm);
          break;

        default:
          ret = RTC_PARAMETERS_INVALID;
          break;
      }

      if ( ret != RTC_SUCCESS )
      {
#warning "If the return code is not RTC_SUCCESS, something needs to be done. Decide whether that \
    Should happen here, in rtc_server, or in each respective thread."
        uart_logger_log_line ("RTC error detected");
        (void) tx_event_flags_set (&error_flags, RTC_ERROR, TX_OR);
//        (void) tx_thread_suspend (this_thread);
      }

      *req.return_code = ret;
      (void) tx_event_flags_set (&rtc_complete_flags, req.complete_flag, TX_OR);
    }

  }
}

/***************************************************************************************************
 ***************************************************************************************************
 ***********************************    Logger Thread    *******************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  Logger thread entry
 *         Logs all system messages to UART port
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void logger_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  logger_message msg;
  UINT tx_ret;
  // Logger stack buffer
  uint8_t logger_stack_buffer[sizeof(log_line_buf) * LOG_QUEUE_LENGTH];
  basic_stack_t logger_stack;
  uart_logger logger;

  uart_logger_init (&logger, &logger_stack, &(logger_stack_buffer[0]), &logger_message_queue,
                    &huart6);

  while ( 1 )
  {
    tx_ret = tx_queue_receive (&logger_message_queue, &msg, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      logger.send_log_line (&(msg.str_buf[0]), msg.strlen);
      logger.return_line_buffer (msg.str_buf);
    }

  }
}

/***************************************************************************************************
 ***************************************************************************************************
 ***********************************    Control Thread    ******************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  Control thread entry
 *         Primary control thread, manages all other threads and meta state
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void control_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  Control control;

  // Run tests if needed
  if ( tests.control_test != NULL )
  {
    tests.control_test (NULL);
  }

  controller_init (&control, &configuration, &thread_handles, &error_flags, &initialization_flags,
                   &irq_flags, &complete_flags, &control_timer, device_handles.battery_adc,
                   &sbd_message);

  watchdog_check_in (CONTROL_THREAD);

  // Run the self test
  if ( !control.startup_procedure () )
  {
    shut_down_all_peripherals ();

    if ( !is_first_sample_window () )
    {
      // Stay stuck here for a minute
      for ( int i = 0; i < 25; i++ )
      {
        watchdog_check_in (CONTROL_THREAD);
        led_sequence (TEST_FAILED_LED_SEQUENCE);
      }
    }

    tx_thread_sleep (1);
    HAL_NVIC_SystemReset ();
  }

  if ( !is_first_sample_window () )
  {
    led_sequence (TEST_PASSED_LED_SEQUENCE);
  }

  while ( 1 )
  {
    watchdog_check_in (CONTROL_THREAD);

#warning "Monitor for errors here and handle them. Monitor for state (watch complete flags) and\
  manage threads appropriately."
    // TODO: Continue with the correct logic, i.e. if GNSS timed out, set alarm and shut down.
    //       Otherwise, continue with other sensors, run waves, etc. Try to keep as many threads
    //       as possible running concurrently.
    // TODO: If a sensor is not present, fill the SBD message fields with 0's.
    // TODO: We're not doing the skip_current_message thing, so if GNSS fails to get a full sample
    //       window, just set the alarm and power down
    // TODO: When GNSS is complete, break
  }

#warning "Make a utility that logs errors into a type 99 message as errors come in. \
  Add timestamps to each error."

}

/***************************************************************************************************
 ***************************************************************************************************
 ************************************    GNSS Thread    ********************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  gnss_thread_entry
 *         Thread that governs the GNSS processing. Note that actual message processing
 *         happens in interrupt context, so this thread is just acting as the traffic cop.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void gnss_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &gnss_thread;
  GNSS gnss;
  uint8_t ubx_message_process_buf[GNSS_MESSAGE_BUF_SIZE];
  uint8_t gnss_config_response_buf[GNSS_CONFIG_BUFFER_SIZE];
  float *north, *east, *down;

  gnss_return_code_t gnss_return_code;
  int number_of_no_sample_errors = 0;
  float last_lat = 0;
  float last_lon = 0;
  uint8_t sbd_port;
  UINT tx_return;
  ULONG actual_flags;
  int timer_ticks_to_get_message = round (
      ((float) TX_TIMER_TICKS_PER_SECOND / (float) configuration.gnss_sampling_rate) + 1);
  uint16_t sample_window_timeout = ((configuration.samples_per_window
                                     / configuration.gnss_sampling_rate)
                                    / 60)
                                   + 2;
  int32_t gnss_max_acq_time = 0;

  // Make sure the waves thread has initialized properly before proceeding
  while ( 1 )
  {
    if ( waves_memory_get_raw_data_pointers (north, east, down) )
    {
      break;
    }
    tx_thread_sleep (1);
  }

  // Init and turn on
  // ** NOTE: RF switch is managed by control thread
  gnss_init (&gnss, &configuration, device_handles.gnss_uart_handle,
             device_handles.gnss_uart_tx_dma_handle, device_handles.gnss_uart_rx_dma_handle,
             &irq_flags, &error_flags, &gnss_timer, &(ubx_message_process_buf[0]),
             &(gnss_config_response_buf[0]), north, east, down);

  gnss.on ();
  // Run tests if needed
  if ( tests.gnss_thread_test != NULL )
  {
    tests.gnss_thread_test (NULL);
  }

  // Apply configuration through UBX_VALSET messages
  if ( !gnss_apply_config (&gnss) )
  {
    gnss_error_out (&gnss, NO_ERROR_FLAG, this_thread, "GNSS failed to initialize.");
  }

  // Report init success
  (void) tx_event_flags_set (&initialization_flags, GNSS_INIT_SUCCESS, TX_OR);
  uart_logger_log_line ("GNSS initialization successful.");

  // This thread has successfully initialized, it should now be checking in with the watchdog
  watchdog_register_thread (GNSS_THREAD);
  watchdog_check_in (GNSS_THREAD);

  // Calculate the max acquisition time if we're running multiple windows per hour
  if ( configuration.windows_per_hour == 1 )
  {
    gnss_max_acq_time = configuration.gnss_max_acquisition_wait_time;
  }
  else if ( configuration.windows_per_hour > 1 )
  {
    if ( is_first_sample_window () )
    {
      gnss_max_acq_time = configuration.gnss_max_acquisition_wait_time;
    }
    else
    {
      gnss_max_acq_time = (60 / configuration.windows_per_hour)
                          - configuration.iridium_max_transmit_time - sample_window_timeout;
    }
  }
  else
  {
    // Invalid (negative) number case
    gnss_max_acq_time = 0;
  }

  // Invalid acquisition time case
  if ( gnss_max_acq_time <= 1 )
  {
    uart_logger_log_line (
        "Invalid GNSS max acquisition time: %d. Check settings for number of samples "
        "per window and windows per hour. Continuing with 5 min acq time.",
        gnss_max_acq_time);

    gnss_max_acq_time = 5;
  }

  watchdog_check_in (GNSS_THREAD);

  // Start the timer for resolution stages
  gnss.start_timer (gnss_max_acq_time);

  // Frame sync and switch to DMA circular mode
  if ( gnss.sync_and_start_reception () != GNSS_SUCCESS )
  {
    // If we were unable to get good GNSS reception and start the DMA transfer loop, then shutdown
    gnss_error_out (&gnss, GNSS_FRAME_SYNC_FAILED, this_thread, "GNSS frame sync failed.");
  }

  // We are now running in DMA circular mode
  uart_logger_log_line ("GNSS successfully switched to circular DMA mode.");

  // Process messages until we have resolved time
  // **NOTE: RTC will be set when time is resolved
  while ( !(gnss.all_resolution_stages_complete || gnss_get_timer_timeout_status ()) )
  {
    watchdog_check_in (GNSS_THREAD);

    tx_return = tx_event_flags_get (&irq_flags, (GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE),
    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {
      gnss.process_message ();
    }
  }

  // Failed to resolve time within alloted period
  if ( gnss_get_timer_timeout_status () )
  {
    gnss_error_out (&gnss, GNSS_RESOLUTION_ERROR, this_thread,
                    "GNSS failed to get a fix within alloted time of :%d.", gnss_max_acq_time);
  }

  // Start the sample window timer
  gnss.stop_timer ();
  gnss.start_timer (sample_window_timeout);

  // Process messages until complete
  while ( !gnss_get_sample_window_complete () )
  {
    watchdog_check_in (GNSS_THREAD);

    tx_return = tx_event_flags_get (&irq_flags, (GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE),
    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {
      gnss.process_message ();
      number_of_no_sample_errors = 0;
    }
    // Message was dropped or incomplete -- replace with running average velocities
    else if ( (tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE) )
    {
      gnss_return_code = gnss.get_running_average_velocities ();

      if ( gnss_return_code == GNSS_NO_SAMPLES_ERROR )
      {
        // If we get a full minute worth of dropped or incomplete messages, fail out
        if ( ++number_of_no_sample_errors == configuration.gnss_sampling_rate * 60 )
        {
          gnss_error_out (&gnss, GNSS_SAMPLE_WINDOW_ERROR, this_thread,
                          "GNSS received too many partial or dropped messages: %d",
                          number_of_no_sample_errors);
        }

      }

    }
    // Any other return code indicates something got corrupted. Let's hope this works...
    else
    {
      gnss_error_out (&gnss, MEMORY_CORRUPTION, this_thread,
                      "Memory corruption detected in GNSS thread.");
    }

    // If this evaluates to true, something hung up with GNSS sampling and we were not able
    // to get all required samples in the alloted time.
    if ( gnss_get_timer_timeout_status () )
    {
      gnss_error_out (&gnss, GNSS_SAMPLE_WINDOW_TIMEOUT, this_thread,
                      "GNSS sample window timed out after %d minutes.", sample_window_timeout);
    }
  }

  watchdog_check_in (GNSS_THREAD);

  // Stop the timer, turn off sensor
  gnss.stop_timer ();
  gnss.off ();

  // Fill in the location fields in the SBD message
  gnss.get_location (&last_lat, &last_lon);
  memcpy (&sbd_message.Lat, &last_lat, sizeof(float));
  memcpy (&sbd_message.Lon, &last_lon, sizeof(float));
  // We're using the "port" field to encode how many samples were averaged divided by 10,
  // up to the limit of an uint8_t
  sbd_port = ((gnss.total_samples_averaged / 10) >= 255) ?
      255 : (gnss.total_samples_averaged / 10);
  memcpy (&sbd_message.port, &sbd_port, sizeof(uint8_t));

  // Deinit -- this will shut down UART port and DMa channels
  gnss_deinit ();

  watchdog_check_in (GNSS_THREAD);
  watchdog_deregister_thread (GNSS_THREAD);

  uart_logger_log_line ("GNSS Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, GNSS_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 **************************************    CT Thread    ********************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  ct_thread_entry
 *         This thread will handle the CT sensor, capture readings, and getting averages..
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void ct_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &ct_thread;
  CT ct;
  ct_sample ct_readings =
    { 0 };
  ct_return_code_t ct_return_code;
  uint32_t ct_parsing_error_counter = 0;
  real16_T half_salinity, half_temp;
  int32_t ct_thread_timeout = TX_TIMER_TICKS_PER_SECOND * 90;

  // Set the mean salinity and temp values to error values in the event the sensor fails
  half_salinity.bitPattern = CT_VALUES_ERROR_CODE;
  half_temp.bitPattern = CT_VALUES_ERROR_CODE;

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  ct_init (&ct, &configuration, device_handles.ct_uart_handle, device_handles.ct_uart_tx_dma_handle,
           device_handles.ct_uart_rx_dma_handle, &ct_uart_sema, &error_flags, &ct_timer);

  ct.on ();

  //
  // Run tests if needed
  if ( tests.ct_thread_test != NULL )
  {
    tests.ct_thread_test (NULL);
  }

  if ( !ct_self_test (&ct, false, &ct_readings) )
  {
    ct_error_out (&ct, NO_ERROR_FLAG, this_thread, "CT self test failed.");
  }

  uart_logger_log_line ("CT initialization complete. Temp = %3f, Salinity = %3f", ct_readings.temp,
                        ct_readings.salinity);
  (void) tx_event_flags_set (&initialization_flags, CT_INIT_SUCCESS, TX_OR);

  // Control will resume when ready
  ct.off ();
  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  ct.on ();
  ct.start_timer (ct_thread_timeout);
  watchdog_register_thread (CT_THREAD);
  watchdog_check_in (CT_THREAD);

  // Turn on the CT sensor, warm it up, and frame sync
  if ( !ct_self_test (&ct, true, &ct_readings) )
  {
    ct_error_out (&ct, CT_ERROR, this_thread, "CT self test failed.");
  }

  // Take our samples
  while ( ct.total_samples < configuration.total_ct_samples )
  {
    watchdog_check_in (CT_THREAD);

    ct_return_code = ct.parse_sample ();

    if ( ct_return_code == CT_PARSING_ERROR )
    {
      ct_parsing_error_counter++;
    }

    if ( (ct_parsing_error_counter >= 10) || (ct_return_code == CT_UART_ERROR) )
    {
      // If there are too many parsing errors or a UART error occurs, stop trying
      ct_error_out (&ct, CT_ERROR, this_thread,
                    "CT sensor too many parsing errors, shutting down.");
    }

    if ( ct_return_code == CT_DONE_SAMPLING )
    {
      break;
    }
  }

  watchdog_check_in (CT_THREAD);

  // Turn off the CT sensor
  ct.stop_timer ();
  ct.off ();

  // Deinit UART and DMA to prevent spurious interrupts
  ct_deinit ();

  // Got our samples, now average them
  ct_return_code = ct.get_averages (&ct_readings);
  // Make sure something didn't go terribly wrong
  if ( ct_return_code == CT_NOT_ENOUGH_SAMPLES )
  {
    ct_error_out (&ct, CT_ERROR, this_thread, "CT sensor did not collect enough samples.");
  }

  // Now set the mean salinity and temp values to the real ones
  half_salinity = doubleToHalf (ct_readings.salinity);
  half_temp = doubleToHalf (ct_readings.temp);

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  watchdog_check_in (CT_THREAD);
  watchdog_deregister_thread (CT_THREAD);

  uart_logger_log_line ("CT Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, CT_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 *********************************    Temperature Thread    ****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  temperature_thread_entry
 *         This thread will handle the temperature sensor, capture readings, and getting averages.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void temperature_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &temperature_thread;
  Temperature temperature;
  temperature_return_code_t temp_return_code = TEMPERATURE_SUCCESS;
  float self_test_reading = 0.0f, sampling_reading = 0.0f;
  real16_T half_temp =
    { 0 };
  int32_t temperature_thread_timeout = TX_TIMER_TICKS_PER_SECOND * 30;
  int32_t fail_counter = 0, max_retries = 10;

  // Set the mean salinity and temp values to error values in the event the sensor fails
  half_temp.bitPattern = TEMPERATURE_VALUES_ERROR_CODE;

  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  temperature_init (&temperature, &configuration, device_handles.core_i2c_handle, &error_flags,
                    &temperature_timer, &core_i2c_mutex, true);

  temperature.on ();

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  if ( !temperature_self_test (&temperature, &self_test_reading) )
  {
    temperature_error_out (&temperature, NO_ERROR_FLAG, this_thread,
                           "Temperature self test failed.");
  }

  uart_logger_log_line ("Temperature initialization complete. Temp =%3f", self_test_reading);
  (void) tx_event_flags_set (&initialization_flags, TEMPERATURE_INIT_SUCCESS, TX_OR);

  temperature.off ();
  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  temperature.on ();
  temperature.start_timer (temperature_thread_timeout);
  watchdog_register_thread (TEMPERATURE_THREAD);
  watchdog_check_in (TEMPERATURE_THREAD);

  while ( fail_counter < max_retries )
  {
    watchdog_check_in (TEMPERATURE_THREAD);

    temp_return_code = temperature.get_readings (false, &sampling_reading);

    if ( temp_return_code == TEMPERATURE_SUCCESS )
    {
      break;
    }

    if ( temperature_get_timeout_status () )
    {
      temperature_error_out (&temperature, TEMPERATURE_ERROR, this_thread,
                             "Temperature thread timed out.");
    }

    fail_counter++;
  }

  if ( fail_counter == max_retries )
  {
    temperature_error_out (
        &temperature, TEMPERATURE_ERROR, this_thread,
        "Unable to get readings from temperature sensor after %d failed attempts.", max_retries);
  }

  half_temp = floatToHalf (sampling_reading);

  temperature.stop_timer ();
  temperature.off ();

  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  watchdog_check_in (TEMPERATURE_THREAD);
  watchdog_deregister_thread (TEMPERATURE_THREAD);

  uart_logger_log_line ("Temperature Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, TEMPERATURE_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 ************************************    Light Thread    *******************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  light_thread_entry
 *         This thread will manage the Light sensor
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void light_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &light_thread;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.light_thread_test != NULL )
  {
    tests.light_thread_test (NULL);
  }

  tx_thread_suspend (this_thread);

  watchdog_register_thread (LIGHT_THREAD);
  watchdog_check_in (LIGHT_THREAD);

  // TODO: Run sensor

  watchdog_check_in (LIGHT_THREAD);
  watchdog_deregister_thread (LIGHT_THREAD);

  uart_logger_log_line ("Light Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, LIGHT_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 **********************************    Turbidity Thread    *****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  turbidity_thread_entry
 *         This thread will manage the turbidity sensor
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void turbidity_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &turbidity_thread;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.turbidity_thread_test != NULL )
  {
    tests.turbidity_thread_test (NULL);
  }

  tx_thread_suspend (this_thread);

  watchdog_register_thread (TURBIDITY_THREAD);
  watchdog_check_in (TURBIDITY_THREAD);

  // TODO: Run sensor

  watchdog_check_in (TURBIDITY_THREAD);
  watchdog_deregister_thread (TURBIDITY_THREAD);

  uart_logger_log_line ("Turbidity Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, TURBIDITY_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 ********************************    Accelerometer Thread    ***************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  accelerometer_thread_entry
 *         This thread will manage the accelerometer sensor
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void accelerometer_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &accelerometer_thread;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.accelerometer_thread_test != NULL )
  {
    tests.accelerometer_thread_test (NULL);
  }

  tx_thread_suspend (this_thread);

  watchdog_register_thread (ACCELEROMETER_THREAD);
  watchdog_check_in (ACCELEROMETER_THREAD);

  // TODO: Run sensor

  watchdog_check_in (ACCELEROMETER_THREAD);
  watchdog_deregister_thread (ACCELEROMETER_THREAD);

  uart_logger_log_line ("Accelerometer Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, ACCELEROMETER_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 *********************************    Expansion Thread 1    ****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  expansion_thread_1_entry
 *         This thread will manage an expansion sensor or module (for future use)
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void expansion_thread_1_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &expansion_thread_1;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.expansion_thread_1_test != NULL )
  {
    tests.expansion_thread_1_test (NULL);
  }

  tx_thread_suspend (this_thread);

//  watchdog_register_thread (EXPANSION_THREAD_1);
//  watchdog_check_in (EXPANSION_THREAD_1);
//
//  // TODO: Run sensor
//
//  watchdog_check_in (EXPANSION_THREAD_1);
//  watchdog_deregister_thread (EXPANSION_THREAD_1);
//
//  uart_logger_log_line ("Expansion Thread 1 complete, now terminating.");
//
//  (void) tx_event_flags_set (&complete_flags, EXPANSION_THREAD_1_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 *********************************    Expansion Thread 2    ****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  expansion_thread_2_entry
 *         This thread will manage an expansion sensor or module (for future use)
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void expansion_thread_2_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &expansion_thread_2;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.expansion_thread_2_test != NULL )
  {
    tests.expansion_thread_2_test (NULL);
  }

  tx_thread_suspend (this_thread);

//  watchdog_register_thread (EXPANSION_THREAD_2);
//  watchdog_check_in (EXPANSION_THREAD_2);
//
//  // TODO: Run sensor
//
//  watchdog_check_in (EXPANSION_THREAD_2);
//  watchdog_deregister_thread (EXPANSION_THREAD_2);
//
//  uart_logger_log_line ("Expansion Thread 2 complete, now terminating.");
//
//  (void) tx_event_flags_set (&complete_flags, EXPANSION_THREAD_2_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 *********************************    Expansion Thread 3    ****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  expansion_thread_3_entry
 *         This thread will manage an expansion sensor or module (for future use)
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void expansion_thread_3_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &expansion_thread_3;

  // TODO: init and self test

  //
  // Run tests if needed
  if ( tests.expansion_thread_3_test != NULL )
  {
    tests.expansion_thread_3_test (NULL);
  }

  tx_thread_suspend (this_thread);
//
//  watchdog_register_thread (EXPANSION_THREAD_3);
//  watchdog_check_in (EXPANSION_THREAD_3);
//
//  // TODO: Run sensor
//
//  watchdog_check_in (EXPANSION_THREAD_3);
//  watchdog_deregister_thread (EXPANSION_THREAD_3);
//
//  uart_logger_log_line ("Expansion Thread 3 complete, now terminating.");
//
//  (void) tx_event_flags_set (&complete_flags, EXPANSION_THREAD_3_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}

/***************************************************************************************************
 ***************************************************************************************************
 ***********************************    NEDWaves Thread    *****************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  waves_thread_entry
 *         This thread will run the NEDWaves algorithm.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void waves_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &waves_thread;
  NEDWaves_memory waves_mem;

  if ( !waves_memory_pool_init (&waves_mem, &configuration, &(waves_byte_pool_buffer[0]),
  WAVES_MEM_POOL_SIZE) )
  {
    uart_logger_log_line ("NED Waves memory pool failed to initialize.");
    tx_thread_suspend (this_thread);
  }

  (void) tx_event_flags_set (&error_flags, WAVES_THREAD_INIT_SUCCESS, TX_OR);
  uart_logger_log_line ("NED Waves initialization successful.");

  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  watchdog_register_thread (WAVES_THREAD);
  watchdog_check_in (WAVES_THREAD);

  //
  // Run tests if needed
  if ( tests.waves_thread_test != NULL )
  {
    tests.waves_thread_test (NULL);
  }

  watchdog_check_in (WAVES_THREAD);

  // Function return parameters
  real16_T E[42];
  real16_T Dp;
  real16_T Hs;
  real16_T Tp;
  real16_T b_fmax;
  real16_T b_fmin;
  signed char a1[42];
  signed char a2[42];
  signed char b1[42];
  signed char b2[42];
  unsigned char check[42];

  /* Call the entry-point 'NEDwaves_memlight'. */
  NEDwaves_memlight (waves_mem.north, waves_mem.east, waves_mem.down,
                     gnss_get_sample_window_frequency (), &Hs, &Tp, &Dp, E, &b_fmin, &b_fmax, a1,
                     b1, a2, b2, check);

  emxDestroyArray_real32_T (waves_mem.north);
  emxDestroyArray_real32_T (waves_mem.east);
  emxDestroyArray_real32_T (waves_mem.down);

  // Done with dynamic memory requirements for NEDWaves, delete the memory pool
  (void) waves_memory_pool_delete ();

  memcpy (&sbd_message.Hs, &Hs, sizeof(real16_T));
  memcpy (&sbd_message.Tp, &Tp, sizeof(real16_T));
  memcpy (&sbd_message.Dp, &Dp, sizeof(real16_T));
  memcpy (&(sbd_message.E_array[0]), &(E[0]), 42 * sizeof(real16_T));
  memcpy (&sbd_message.f_min, &b_fmin, sizeof(real16_T));
  memcpy (&sbd_message.f_max, &b_fmax, sizeof(real16_T));
  memcpy (&(sbd_message.a1_array[0]), &(a1[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.b1_array[0]), &(b1[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.a2_array[0]), &(a2[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.b2_array[0]), &(b2[0]), 42 * sizeof(signed char));
  memcpy (&(sbd_message.cf_array[0]), &(check[0]), 42 * sizeof(unsigned char));

  watchdog_check_in (WAVES_THREAD);
  watchdog_deregister_thread (WAVES_THREAD);

  uart_logger_log_line ("NEDWaves Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, WAVES_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}
/***************************************************************************************************
 ***************************************************************************************************
 ***********************************    Iridium Thread    ******************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  iridium_thread_entry
 *         This thread will handle message sending via Iridium modem.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void iridium_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &iridium_thread;
  Iridium iridium;
  iridium_return_code_t iridium_return_code = IRIDIUM_SUCCESS;
  int32_t iridium_thread_timeout = configuration.iridium_max_transmit_time;
  UINT tx_return;
  int fail_counter;
  char ascii_7 = '7';
  uint8_t sbd_type = 52;
  uint16_t sbd_size = 327;
  uint32_t sbd_timestamp = 0;
  struct tm time_struct =
    { 0 };
  time_t time_now = 0;

  iridium_init (&iridium, &configuration, device_handles.iridium_uart_handle,
                device_handles.iridium_minutes_timer, &thread_control_flags, &error_flags,
                &sbd_message, &(iridium_error_message[0]), &(iridium_response_message[0]),
                &sbd_message_queue);

  iridium.on ();
  iridium.wake ();

  if ( !iridium_apply_config (&iridium) )
  {
    iridium_error_out (&iridium, NO_ERROR_FLAG, this_thread, "Iridium modem failed to initialize.");
  }

  uart_logger_log_line ("Iridium modem initialized successfully.");
  (void) tx_event_flags_set (&initialization_flags, IRIDIUM_INIT_SUCCESS, TX_OR);

  iridium.charge_caps (IRIDIUM_INITIAL_CAP_CHARGE_TIME);
  iridium.sleep ();

  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  iridium.wake ();

  watchdog_register_thread (IRIDIUM_THREAD);
  watchdog_check_in (IRIDIUM_THREAD);

  iridium.start_timer (iridium_thread_timeout);

  //
  // Run tests if needed
  if ( tests.iridium_thread_test != NULL )
  {
    tests.iridium_thread_test (NULL);
  }

  // finish filling out the SBD message
  rtc_server_get_time (&time_struct, IRIDIUM_THREAD_COMPLETE);
  time_now = mktime (&time_struct);
  sbd_timestamp = (uint32_t) time_now;
  memcpy (&sbd_message.legacy_number_7, &ascii_7, sizeof(char));
  memcpy (&sbd_message.type, &sbd_type, sizeof(uint8_t));
  memcpy (&sbd_message.size, &sbd_size, sizeof(uint16_t));
  memcpy (&sbd_message.timestamp, &sbd_timestamp, sizeof(float));

  if ( !iridium_apply_config (&iridium) )
  {
    // Need to save the message
    persistent_storage_save_iridium_message (&sbd_message);
    iridium_error_out (&iridium, NO_ERROR_FLAG, this_thread, "Iridium modem failed to initialize.");
  }

  watchdog_check_in (IRIDIUM_THREAD);

#warning "Put this in a loop checking for timeout condition. Also need to manage queued \
  message sending here as transmit_message is greatly simplified."

  iridium.transmit_message (&sbd_message);

  watchdog_check_in (IRIDIUM_THREAD);

#warning "Figure out error message reporting here."

  // Turn off the modem
  iridium.stop_timer ();
  iridium.sleep ();
  iridium.off ();

  iridium_deinit ();

  watchdog_check_in (IRIDIUM_THREAD);
  watchdog_deregister_thread (IRIDIUM_THREAD);

  uart_logger_log_line ("Iridium Thread complete, now terminating.");

  (void) tx_event_flags_set (&complete_flags, IRIDIUM_THREAD_COMPLETE, TX_OR);
  tx_thread_terminate (this_thread);
}
/* USER CODE END 1 */

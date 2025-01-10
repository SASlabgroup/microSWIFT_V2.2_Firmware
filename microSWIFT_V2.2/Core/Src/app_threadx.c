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
#include "stdio.h"
#include "app_filex.h"
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
#include "light_sensor.h"
#include "turbidity_sensor.h"
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
#include "leds.h"
#include "shared_i2c_bus.h"

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
TX_THREAD led_thread;
TX_THREAD i2c_bus_thread;
TX_THREAD logger_thread;
TX_THREAD gnss_thread;
TX_THREAD ct_thread;
TX_THREAD temperature_thread;
TX_THREAD light_thread;
TX_THREAD turbidity_thread;
TX_THREAD waves_thread;
TX_THREAD iridium_thread;
// @formatter:off
Thread_Handles thread_handles =
  {
    &control_thread,
    &rtc_thread,
    &led_thread,
    &i2c_bus_thread,
    &logger_thread,
    &gnss_thread,
    &ct_thread,
    &temperature_thread,
    &light_thread,
    &turbidity_thread,
    &waves_thread,
    &iridium_thread,
    &fx_thread
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
// RTC complete flags (for RTC server)
TX_EVENT_FLAGS_GROUP rtc_complete_flags;
// I2C complete flags
TX_EVENT_FLAGS_GROUP i2c_complete_flags;
// Flags for which thread is checkin in with watchdog
TX_EVENT_FLAGS_GROUP watchdog_check_in_flags;
// Timers for threads
TX_TIMER led_duration_timer;
TX_TIMER control_timer;
TX_TIMER gnss_timer;
TX_TIMER ct_timer;
TX_TIMER temperature_timer;
TX_TIMER light_timer;
TX_TIMER turbidity_timer;
TX_TIMER waves_timer;
TX_TIMER iridium_timer;
// Comms buses semaphores !! No GNSS UART sema as it uses event flags instead
TX_SEMAPHORE ext_rtc_spi_sema;
TX_SEMAPHORE core_i2c_sema;
TX_SEMAPHORE iridium_uart_sema;
TX_SEMAPHORE ct_uart_sema;
TX_SEMAPHORE logger_sema;
// Logger mutex
TX_MUTEX logger_mutex;
// Server/client message queue for RTC (including watchdog function)
TX_QUEUE rtc_messaging_queue;
// Logger message passing queue
TX_QUEUE logger_message_queue;
// Queue for LED thread
TX_QUEUE led_queue;
// Shared I2C bus queue
TX_QUEUE i2c_bus_queue;

// Buffer for the Waves byte pool to allow dynamic memory allocation in a bounded fashion
__ALIGN_BEGIN UCHAR waves_byte_pool_buffer[WAVES_MEM_POOL_SIZE] __attribute__((section(".ram1")))__ALIGN_END;
TX_BYTE_POOL waves_byte_pool;

// Logger block pool
__ALIGN_BEGIN UCHAR logger_block_buffer[(sizeof(log_line_buf) * LOG_QUEUE_LENGTH)
                                        + (LOG_QUEUE_LENGTH * sizeof(void*))] __attribute__((section(".ram1")))__ALIGN_END;

TX_BLOCK_POOL logger_block_pool;

// Light sensor sample buffer
ALIGN_32BYTES(
    light_basic_counts light_sensor_sample_buffer[LIGHT_SENSOR_BYTE_POOL_BUFFER_SIZE / sizeof(light_basic_counts)] __attribute__((section(".ram1"))));

// Turbidity sensor sample buffer
ALIGN_32BYTES(
    uint16_t turbidity_sensor_ambient_buffer[TURBIDITY_SENSOR_SAMPLE_BUFFER_SIZE / sizeof(uint16_t)] __attribute__((section(".ram1"))));
ALIGN_32BYTES(
    uint16_t turbidity_sensor_proximity_buffer[TURBIDITY_SENSOR_SAMPLE_BUFFER_SIZE / sizeof(uint16_t)] __attribute__((section(".ram1"))));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

// Threads
static void rtc_thread_entry ( ULONG thread_input );
static void logger_thread_entry ( ULONG thread_input );
static void control_thread_entry ( ULONG thread_input );
static void led_thread_entry ( ULONG thread_input );
static void i2c_bus_thread_entry ( ULONG thread_input );
static void gnss_thread_entry ( ULONG thread_input );
static void waves_thread_entry ( ULONG thread_input );
static void iridium_thread_entry ( ULONG thread_input );
static void ct_thread_entry ( ULONG thread_input );
static void temperature_thread_entry ( ULONG thread_input );
static void light_thread_entry ( ULONG thread_input );
static void turbidity_thread_entry ( ULONG thread_input );

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
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the rtc thread. VERY_HIGH priority level and no preemption possible
  ret = tx_thread_create(&rtc_thread, "rtc thread", rtc_thread_entry, 0, pointer, M_STACK,
                         VERY_HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the LED thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XXS_STACK,
  TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the logger thread. Low priority priority level and no preemption possible
  ret = tx_thread_create(&led_thread, "LED thread", led_thread_entry, 0, pointer, XXS_STACK,
                         LOW_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the LED thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XS_STACK,
  TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the I2C thread, VERY_HIGH_PRIORITY level and no preemption possible
  ret = tx_thread_create(&i2c_bus_thread, "I2C bus thread", i2c_bus_thread_entry, 0, pointer,
                         XS_STACK, VERY_HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the logger thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK,
  TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the logger thread. Low priority priority level and no preemption possible
  ret = tx_thread_create(&logger_thread, "logger thread", logger_thread_entry, 0, pointer, M_STACK,
                         LOW_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the gnss thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XL_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the gnss thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&gnss_thread, "gnss thread", gnss_thread_entry, 0, pointer, XL_STACK,
                         HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
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
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the temperature thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&temperature_thread, "temperature thread", temperature_thread_entry, 0,
                         pointer, M_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE,
                         TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the light thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XL_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the light thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&light_thread, "light thread", light_thread_entry, 0, pointer, XL_STACK,
                         MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  //
  // Allocate stack for the turbidity thread
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, XL_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the turbidity thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&turbidity_thread, "turbidity thread", turbidity_thread_entry, 0, pointer,
                         XL_STACK, MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
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
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, L_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the Iridium thread. HIGH priority, no preemption-threshold
  ret = tx_thread_create(&iridium_thread, "iridium thread", iridium_thread_entry, 0, pointer,
                         L_STACK, HIGH_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ********************************** Byte and Block Pools ****************************************
   ************************************************************************************************/
  //
  // Block pool for the UART logger
  ret = tx_block_pool_create(&logger_block_pool, "UART Logger Block Pool", sizeof(log_line_buf),
                             &logger_block_buffer[0], sizeof(logger_block_buffer));
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  /* Note: the Waves thread will create and manage its own block pool. */

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
  // Create the I2C complete flags we'll use for tracking I2C function completion
  ret = tx_event_flags_create(&i2c_complete_flags, "I2C complete flags");
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
  ret = tx_timer_create(&led_duration_timer, "LED duration timer", led_timer_expired, 0, 1, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&control_timer, "Control thread timer", control_timer_expired, 0, 1, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&gnss_timer, "GNSS thread timer", gnss_timer_expired, 0, 1, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&ct_timer, "CT thread timer", ct_timer_expired, 0, 1, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&temperature_timer, "Temperature thread timer", temperature_timer_expired,
                        0, 1, 0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&light_timer, "Light thread timer", light_timer_expired, 0, 1, 0,
                        TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&turbidity_timer, "Turbidity thread timer", turbidity_timer_expired, 0, 1,
                        0, TX_NO_ACTIVATE);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_timer_create(&iridium_timer, "Iridium thread timer", iridium_timer_expired, 0, 1, 0,
                        TX_NO_ACTIVATE);
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

  ret = tx_semaphore_create(&core_i2c_sema, "Core I2C sema", 0);
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

  ret = tx_semaphore_create(&logger_sema, "Logger UART sema", 0);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   **************************************** Mutexes ***********************************************
   ************************************************************************************************/

  ret = tx_mutex_create(&logger_mutex, "UART Logger mutex", TX_NO_INHERIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ************************************** Message Queues ******************************************
   ************************************************************************************************/
  // Server message queue for UART Logger
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

  // Server message queue for RTC (including watchdog function)
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
                          sizeof(rtc_request_message) * RTC_QUEUE_LENGTH, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&rtc_messaging_queue, "RTC msg queue",
                        sizeof(rtc_request_message) / sizeof(uint32_t), pointer,
                        sizeof(rtc_request_message) * RTC_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
                          sizeof(led_message) * LED_MESSAGE_QUEUE_LENGTH, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&led_queue, "LED queue", sizeof(led_message) / sizeof(uint32_t), pointer,
                        sizeof(led_message) * LED_MESSAGE_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer,
                          sizeof(i2c_queue_message) * I2C_QUEUE_LENGTH, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  ret = tx_queue_create(&i2c_bus_queue, "I2C Bus queue",
                        sizeof(i2c_queue_message) / sizeof(uint32_t), pointer,
                        sizeof(i2c_queue_message) * I2C_QUEUE_LENGTH);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }

  /************************************************************************************************
   ***************************************** Misc init ********************************************
   ************************************************************************************************/
  device_handles.core_spi_handle = &hspi1;
  device_handles.core_i2c_handle = &hi2c2;
  device_handles.iridium_uart_handle = &huart4;
  device_handles.gnss_uart_handle = &huart2;
  device_handles.ct_uart_handle = &huart1;
  device_handles.logger_uart_handle = &huart3;
  device_handles.ext_psram_handle = &hospi1;
  device_handles.battery_adc = &hadc1;
  device_handles.gnss_uart_tx_dma_handle = &handle_GPDMA1_Channel7;
  device_handles.gnss_uart_rx_dma_handle = &handle_GPDMA1_Channel6;
  device_handles.iridium_uart_tx_dma_handle = &handle_GPDMA1_Channel9;
  device_handles.iridium_uart_rx_dma_handle = &handle_GPDMA1_Channel8;
  device_handles.ct_uart_tx_dma_handle = &handle_GPDMA1_Channel11;
  device_handles.ct_uart_rx_dma_handle = &handle_GPDMA1_Channel10;
  device_handles.logger_uart_tx_dma_handle = &handle_GPDMA1_Channel3;
  device_handles.logger_uart_rx_dma_handle = &handle_GPDMA1_Channel2;

  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.duty_cycle = DUTY_CYCLE_PERIOD;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
  configuration.total_ct_samples = TOTAL_CT_SAMPLES;
  configuration.total_temp_samples = TOTAL_TEMPERATURE_SAMPLES;
  configuration.total_light_samples = TOTAL_LIGHT_SAMPLES;
  configuration.total_turbidity_samples = TOTAL_TURBIDITY_SAMPLES;
  configuration.iridium_v3f = IRIDIUM_V3F;
  configuration.gnss_high_performance_mode = GNSS_HIGH_PERFORMANCE_MODE_ENABLED;
  configuration.ct_enabled = CT_ENABLED;
  configuration.temperature_enabled = TEMPERATURE_ENABLED;
  configuration.light_enabled = LIGHT_SENSOR_ENABLED;
  configuration.turbidity_enabled = TURBIDITY_SENSOR_ENABLED;
  configuration.gnss_max_acquisition_wait_time = get_gnss_acquisition_timeout (&configuration);

  /* USER CODE END App_ThreadX_MEM_POOL */
  /* USER CODE BEGIN App_ThreadX_Init */
  //
  // Run tests if needed
  if ( tests.threadx_init_test != NULL )
  {
    tests.threadx_init_test (NULL);
  }

  persistent_ram_init ();
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
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter ();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
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
static uint32_t watchdog_refresh_counter = 0; // debugging
static void rtc_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = tx_thread_identify ();
  Ext_RTC rtc =
    { 0 };
  uSWIFT_return_code_t ret;
  UINT tx_ret;
  rtc_request_message req;

  tx_thread_sleep (10);

  ret = ext_rtc_init (&rtc, device_handles.core_spi_handle, &ext_rtc_spi_sema);

  rtc_server_init (&rtc_messaging_queue, &rtc_complete_flags);

  // Setup the RTC
  ret |= rtc.setup_rtc ();

  // Initialize the watchdog
  ret |= rtc.config_watchdog (WATCHDOG_PERIOD);

  if ( ret != uSWIFT_SUCCESS )
  {
    // Gotta wait for the logger to fully initialize
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
    rtc_error_out (this_thread, "RTC failed to initialize.");
  }

  // Set the GPIO pin low for the OR logic gate
  HAL_GPIO_WritePin (WDOG_OR_INPUT_GPIO_Port, WDOG_OR_INPUT_Pin, GPIO_PIN_RESET);

  (void) tx_event_flags_set (&initialization_flags, RTC_INIT_SUCCESS, TX_OR);

  LOG("RTC Initialization successful.");

  while ( 1 )
  {
    // See if we have any requests on the queue
    tx_ret = tx_queue_receive (&rtc_messaging_queue, &req, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      switch ( req.request )
      {
        case REFRESH_WATCHDOG:
          watchdog_refresh_counter++;
          ret = rtc.refresh_watchdog ();
          break;

        case GET_TIME:
          ret = rtc.get_date_time (req.input_output_struct.get_set_time.time_struct);
          break;

        case SET_TIME:

          ret = rtc.set_date_time (req.input_output_struct.get_set_time.time_struct);
          break;

        case SET_TIMESTAMP:
          ret = rtc.set_timestamp (req.input_output_struct.get_set_timestamp.which_timestamp);
          break;

        case GET_TIMESTAMP:
          ret = rtc.get_timestamp (req.input_output_struct.get_set_timestamp.which_timestamp,
                                   &req.input_output_struct.get_set_timestamp.timestamp);
          break;

        case SET_ALARM:
          ret = rtc.set_alarm (req.input_output_struct.set_alarm);
          break;

        case CLEAR_FLAG:
          ret = rtc.clear_flag (req.input_output_struct.clear_flag);
          break;

        default:
          ret = uSWIFT_PARAMETERS_INVALID;
          break;
      }

      if ( ret != uSWIFT_SUCCESS )
      {
        rtc_error_out (this_thread, "RTC failed to service request %d, returning code %d.",
                       (int) req.request, (int) ret);
      }

      if ( req.return_code != NULL )
      {
        *req.return_code = ret;
      }
      (void) tx_event_flags_set (&rtc_complete_flags, req.complete_flag, TX_OR);
    }
  }
}

/***************************************************************************************************
 ***************************************************************************************************
 *************************************    LED Thread    ********************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  LED thread entry
 *         Manages LED status
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void led_thread_entry ( ULONG thread_input )
{
  UINT tx_ret;
  led_message msg;
  LEDs leds;

  leds_init (&leds, &led_duration_timer, &led_queue);

  while ( 1 )
  {
    tx_ret = tx_queue_receive (&led_queue, &msg, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      leds.play_sequence (msg.sequence, msg.duration_sec);
    }
  }
}

/***************************************************************************************************
 ***************************************************************************************************
 ***********************************    I2C Bus Thread    ******************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 * @brief  I2C Bus thread entry
 *         Manages the shared I2C bus
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void i2c_bus_thread_entry ( ULONG thread_input )
{
  TX_THREAD *this_thread = tx_thread_identify ();
  Shared_I2C_Bus shared_bus;
  i2c_queue_message incoming_msg =
    { 0 };
  uSWIFT_return_code_t ret;
  UINT tx_ret;

  if ( !shared_i2c_init (&shared_bus, &hi2c2, &core_i2c_sema) )
  {
    i2c_error_out (this_thread, "Core I2C Bus failed to initialize.");
  }

  shared_i2c_server_init (&i2c_bus_queue, &i2c_complete_flags);

  while ( 1 )
  {
    // See if we have any requests on the queue
    tx_ret = tx_queue_receive (&i2c_bus_queue, &incoming_msg, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      switch ( incoming_msg.operation_type )
      {
        case I2C_READ:
          ret = shared_bus.read (incoming_msg.dev_addr, incoming_msg.dev_reg,
                                 incoming_msg.input_output_buffer, incoming_msg.data_len,
                                 I2C_OP_WAIT_TICKS);
          break;

        case I2C_WRITE:
          ret = shared_bus.write (incoming_msg.dev_addr, incoming_msg.dev_reg,
                                  incoming_msg.input_output_buffer, incoming_msg.data_len,
                                  I2C_OP_WAIT_TICKS);
          break;

        default:
          ret = uSWIFT_PARAMETERS_INVALID;
          break;
      }

      if ( ret != uSWIFT_SUCCESS )
      {
        i2c_error_out (this_thread, "I2C Bus failed to service request %d, returning code %d.",
                       (int) incoming_msg.operation_type, (int) ret);
      }

      if ( incoming_msg.return_code != NULL )
      {
        *incoming_msg.return_code = ret;
      }

      (void) tx_event_flags_set (&i2c_complete_flags, incoming_msg.complete_flag, TX_OR);
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
  uart_logger logger =
    { 0 };
  ULONG time_start, time_end, time_elapsed;

  uart_logger_init (&logger, &logger_block_pool, &logger_message_queue, &logger_mutex,
                    device_handles.logger_uart_handle);

  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

  while ( 1 )
  {
    tx_ret = tx_queue_receive (&logger_message_queue, &msg, TX_WAIT_FOREVER);
    if ( tx_ret == TX_SUCCESS )
    {
      time_start = tx_time_get ();

      logger.send_log_line (&(msg.str_buf[0]), msg.strlen);

      // Pass the buffer down to the file system for saving to SD card
      (void) file_system_server_save_log_line (&(msg.str_buf[0]));

      logger.return_line_buffer (msg.str_buf);

      // Need to wait until the transmission is complete before grabbing another message
      (void) tx_semaphore_get (&logger_sema, TX_WAIT_FOREVER);

      // We only know the DMA complete time, not the time it has completed Tx on the wire
      time_end = tx_time_get ();

      time_elapsed = (time_end - time_start);

      if ( time_elapsed < LOGGER_MAX_TICKS_TO_TX_MSG )
      {
        tx_thread_sleep (LOGGER_MAX_TICKS_TO_TX_MSG - time_elapsed);
      }
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
  Control control =
    { 0 };
  struct watchdog_t watchdog;
  bool first_window = is_first_sample_window (), heartbeat_started = false;
  uint32_t watchdog_counter = 0;
  ULONG start_time = 0;

  // Run tests if needed
  if ( tests.control_test != NULL )
  {
    tests.control_test (NULL);
  }

  // Let RTC and Logger thread init
  tx_thread_sleep (10);

  controller_init (&control, &configuration, &thread_handles, &error_flags, &initialization_flags,
                   &irq_flags, &complete_flags, &control_timer, device_handles.battery_adc,
                   &sbd_message);

  LOG("\n\nBoot.");

  if ( watchdog_init (&watchdog, &watchdog_check_in_flags) != WATCHDOG_OK )
  {
    HAL_NVIC_SystemReset ();
  }

  // Run the self test
  if ( !control.startup_procedure () )
  {
#warning "Add a log that displays the reason self test failed."
    if ( first_window )
    {
      control.shutdown_all_peripherals ();

      led_light_sequence (TEST_FAILED_LED_SEQUENCE, LED_SEQUENCE_FOREVER);
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND * 30);

      HAL_NVIC_SystemReset ();
    }
  }

  watchdog_check_in (CONTROL_THREAD);

  if ( first_window )
  {
    led_light_sequence (TEST_PASSED_LED_SEQUENCE, 10);
  }
  else
  {
    led_light_sequence (HEARTBEAT_SEQUENCE, LED_SEQUENCE_FOREVER);
    heartbeat_started = true;
  }

  start_time = tx_time_get ();

  while ( 1 )
  {
    if ( ++watchdog_counter % TX_TIMER_TICKS_PER_SECOND == 0 )
    {
      watchdog_check_in (CONTROL_THREAD);
    }

    control.monitor_and_handle_errors ();
    control.manage_state ();

    // Make sure we haven't timeout for the full duty cycle
    if ( control_get_timeout_status () )
    {
      LOG("Duty cycle timeout. Starting shutdown sequence.");
      tx_thread_sleep (1);
      control.shutdown_procedure ();
    }

    if ( ((tx_time_get () - start_time) > (TX_TIMER_TICKS_PER_SECOND * 10)) && !heartbeat_started )
    {
      led_light_sequence (HEARTBEAT_SEQUENCE, LED_SEQUENCE_FOREVER);
      heartbeat_started = true;
    }

    tx_thread_sleep (1);
  }

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
  TX_THREAD *this_thread = tx_thread_identify ();
  GNSS gnss =
    { 0 };
  float *north = NULL, *east = NULL, *down = NULL;
  uSWIFT_return_code_t gnss_return_code;
  int number_of_no_sample_errors = 0;
  float last_lat = 0;
  float last_lon = 0;
  uint8_t sbd_port;
  UINT tx_return;
  ULONG actual_flags;
  int timer_ticks_to_get_message = round (
      ((float) TX_TIMER_TICKS_PER_SECOND / (float) configuration.gnss_sampling_rate) + 25);
  uint32_t two_mins_remaining_sample_count = abs (
      (2 * 60 * configuration.gnss_sampling_rate) - configuration.samples_per_window);
  bool two_mins_out_msg_sent = false, start_flag_sent = false;
  uint32_t total_samples = 0;

  if ( usart2_init () != UART_OK )
  {
    gnss_error_out (&gnss, GNSS_INIT_FAILED, this_thread, "GNSS UART port failed to initialize.");
  }

  // Make sure the waves thread has initialized properly before proceeding
  while ( 1 )
  {
    if ( waves_memory_get_raw_data_pointers (&north, &east, &down) )
    {
      break;
    }
    tx_thread_sleep (1);
  }

  // Init and turn on
  // ** NOTE: RF switch is managed by control thread
  gnss_init (&gnss, &configuration, device_handles.gnss_uart_handle,
             device_handles.gnss_uart_tx_dma_handle, device_handles.gnss_uart_rx_dma_handle,
             &irq_flags, &error_flags, &gnss_timer, north, east, down);

  gnss.on ();
  // Run tests if needed
  if ( tests.gnss_thread_test != NULL )
  {
    tests.gnss_thread_test (&gnss);
  }

  // Apply configuration through UBX_VALSET messages
  if ( !gnss_apply_config (&gnss) )
  {
    gnss_error_out (&gnss, GNSS_CONFIGURATION_FAILED, this_thread, "GNSS failed to initialize.");
  }

  // Report init success
  (void) tx_event_flags_set (&initialization_flags, GNSS_INIT_SUCCESS, TX_OR);
  LOG("GNSS initialization successful.");

  // This thread has successfully initialized, it should now be checking in with the watchdog
  watchdog_register_thread (GNSS_THREAD);
  watchdog_check_in (GNSS_THREAD);

  // Start the timer for resolution stages
  gnss.start_timer (configuration.gnss_max_acquisition_wait_time);

  // Frame sync and switch to DMA circular mode
  if ( gnss.sync_and_start_reception () != uSWIFT_SUCCESS )
  {
    // If we were unable to get good GNSS reception and start the DMA transfer loop, then shutdown
    gnss_error_out (&gnss, GNSS_FRAME_SYNC_FAILED, this_thread, "GNSS frame sync failed.");
  }

  // We are now running in DMA circular mode
  LOG("GNSS successfully switched to circular DMA mode.");

  // Process messages until we have resolved time
  // **NOTE: RTC will be set when time is resolved
  while ( !(gnss.all_resolution_stages_complete || gnss_get_timer_timeout_status ()) )
  {
    if ( gnss.total_samples % 50 == 0 )
    {
      watchdog_check_in (GNSS_THREAD);
    }

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
                    "GNSS failed to get a fix within alloted time of %d minutes.",
                    configuration.gnss_max_acquisition_wait_time);
  }

  // Start the sample window timer
  gnss.stop_timer ();
  gnss.start_timer (get_gnss_sample_window_timeout (&configuration));

  // Process messages until complete
  while ( !gnss_get_sample_window_complete () )
  {
    total_samples = gnss_get_samples_processed ();

    if ( total_samples % (configuration.gnss_sampling_rate * 10) == 0 )
    {
      watchdog_check_in (GNSS_THREAD);
    }

    if ( (total_samples > 0) && (!start_flag_sent) && gnss.current_fix_is_good )
    {
      (void) tx_event_flags_set (&complete_flags, GNSS_SAMPLING_STARTED_FIX_GOOD, TX_OR);
      start_flag_sent = true;
    }

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

      if ( gnss_return_code == uSWIFT_NO_SAMPLES_ERROR )
      {
        // If we get a full minute worth of dropped or incomplete messages, fail out
        if ( ++number_of_no_sample_errors == configuration.gnss_sampling_rate * 60 )
        {
          gnss_error_out (&gnss, GNSS_TOO_MANY_PARTIAL_MSGS, this_thread,
                          "GNSS received too many partial or dropped messages: %d",
                          number_of_no_sample_errors);
        }

      }

    }
    // Any other return code indicates something got corrupted. Let's hope this works...
    else
    {
      gnss_error_out (&gnss, MEMORY_CORRUPTION_ERROR, this_thread,
                      "Memory corruption detected in GNSS thread.");
    }

    // If this evaluates to true, something hung up with GNSS sampling and we were not able
    // to get all required samples in the alloted time.
    if ( gnss_get_timer_timeout_status () )
    {
      gnss_error_out (&gnss, GNSS_SAMPLE_WINDOW_TIMEOUT, this_thread,
                      "GNSS sample window timed out after %d minutes.",
                      get_gnss_sample_window_timeout (&configuration));
    }

    // Check if we are two mins out
    if ( !two_mins_out_msg_sent
         && (gnss_get_samples_processed () >= two_mins_remaining_sample_count) )
    {
      (void) tx_event_flags_set (&complete_flags, GNSS_TWO_MINS_OUT_FROM_COMPLETION, TX_OR);
      two_mins_out_msg_sent = true;
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

  //  sbd_port = ((gnss.total_samples_averaged / 10) >= 255) ?
  //      255 : (gnss.total_samples_averaged / 10);

  // We were using the "port" field to encode how many samples were averaged divided by 10, but
  // now we are using it to store the firmware version
  sbd_port = *((uint8_t*) &firmware_version);
  memcpy (&sbd_message.port, &sbd_port, sizeof(uint8_t));

  // Deinit -- this will shut down UART port and DMa channels
  gnss_deinit ();

  watchdog_check_in (GNSS_THREAD);

  (void) file_system_server_save_gnss_raw (&gnss);
  (void) file_system_server_save_gnss_track (&gnss);

  watchdog_check_in (GNSS_THREAD);
  watchdog_deregister_thread (GNSS_THREAD);

  // The logger gets weird if there is no break here...
  tx_thread_sleep (25);

  (void) tx_event_flags_set (&complete_flags, GNSS_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  TX_THREAD *this_thread = tx_thread_identify ();
  CT ct =
    { 0 };
  ct_sample ct_readings =
    { 0 };
  uSWIFT_return_code_t ct_return_code;
  uint32_t ct_parsing_error_counter = 0;
  real16_T half_salinity, half_temp;
  int32_t ct_thread_timeout = 3; // mins

  tx_thread_sleep (10);

  if ( usart1_init () != UART_OK )
  {
    ct_error_out (&ct, CT_INIT_FAILED, this_thread, "CT UART port failed to initialize.");
  }

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
    ct_error_out (&ct, CT_INIT_FAILED, this_thread, "CT self test failed.");
  }

  LOG("CT initialization complete. Temp = %3f, Salinity = %3f", ct_readings.temp,
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
    ct_error_out (&ct, CT_INIT_FAILED, this_thread, "CT self test failed.");
  }

  // Take our samples
  while ( 1 )
  {
    watchdog_check_in (CT_THREAD);

    ct_return_code = ct.parse_sample ();

    if ( ct_return_code == uSWIFT_PROCESSING_ERROR )
    {
      ct_parsing_error_counter++;
    }

    if ( (ct_parsing_error_counter >= 10) || (ct_return_code == uSWIFT_IO_ERROR) )
    {
      // If there are too many parsing errors or a UART error occurs, stop trying
      ct_error_out (&ct, CT_SAMPLING_ERROR, this_thread,
                    "CT sensor too many parsing errors, shutting down.");
    }

    // If this evaluates to true, something hung up with GNSS sampling and we were not able
    // to get all required samples in the alloted time.
    if ( ct_get_timeout_status () )
    {
      ct_error_out (&ct, CT_SAMPLE_WINDOW_TIMEOUT, this_thread,
                    "CT sample window timed out after %d minutes.", ct_thread_timeout);
    }

    if ( ct_return_code == uSWIFT_DONE_SAMPLING )
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
  if ( ct_return_code == uSWIFT_NO_SAMPLES_ERROR )
  {
    ct_error_out (&ct, CT_SAMPLING_ERROR, this_thread, "CT sensor did not collect enough samples.");
  }

  // Now set the mean salinity and temp values to the real ones
  half_salinity = doubleToHalf (ct_readings.salinity);
  half_temp = doubleToHalf (ct_readings.temp);

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  watchdog_check_in (CT_THREAD);

  (void) file_system_server_save_ct_raw (&ct);

  watchdog_check_in (CT_THREAD);
  watchdog_deregister_thread (CT_THREAD);

  (void) tx_event_flags_set (&complete_flags, CT_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  TX_THREAD *this_thread = tx_thread_identify ();
  Temperature temperature =
    { 0 };
  uSWIFT_return_code_t temp_return_code = uSWIFT_SUCCESS;
  float self_test_reading = 0.0f, sampling_reading = 0.0f;
  real16_T half_temp =
    { 0 };
  int32_t temperature_thread_timeout = 2; // minutes
  int32_t fail_counter = 0, max_retries = 10;

  tx_thread_sleep (10);

  // Set the mean salinity and temp values to error values in the event the sensor fails
  half_temp.bitPattern = TEMPERATURE_VALUES_ERROR_CODE;

  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  temperature_init (&temperature, &configuration, &error_flags, &temperature_timer, true);

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  if ( !temperature_self_test (&temperature, &self_test_reading) )
  {
    temperature_error_out (&temperature, TEMPERATURE_INIT_FAILED, this_thread,
                           "Temperature self test failed.");
  }

  LOG("Temperature initialization complete. Temp = %4.3f degC, %4.3f degF.", self_test_reading,
      ((self_test_reading * 9) / 5) + 32);
  (void) tx_event_flags_set (&initialization_flags, TEMPERATURE_INIT_SUCCESS, TX_OR);

  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  temperature.start_timer (temperature_thread_timeout);
  watchdog_register_thread (TEMPERATURE_THREAD);
  watchdog_check_in (TEMPERATURE_THREAD);

  while ( fail_counter < max_retries )
  {
    watchdog_check_in (TEMPERATURE_THREAD);

    temp_return_code = temperature.get_readings (false, &sampling_reading);

    if ( temp_return_code == uSWIFT_SUCCESS )
    {
      break;
    }

    if ( temperature_get_timeout_status () )
    {
      temperature_error_out (&temperature, TEMPERATURE_SAMPLE_WINDOW_TIMEOUT, this_thread,
                             "Temperature sample window timed out after %d minutes.",
                             temperature_thread_timeout);
    }

    fail_counter++;
  }

  if ( fail_counter == max_retries )
  {
    temperature_error_out (
        &temperature, TEMPERATURE_SAMPLING_ERROR, this_thread,
        "Unable to get readings from temperature sensor after %d failed attempts.", max_retries);
  }

  half_temp = floatToHalf (sampling_reading);

  temperature.stop_timer ();

  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  watchdog_check_in (TEMPERATURE_THREAD);

  (void) file_system_server_save_ct_raw (&temperature);

  watchdog_check_in (TEMPERATURE_THREAD);
  watchdog_deregister_thread (TEMPERATURE_THREAD);

  (void) tx_event_flags_set (&complete_flags, TEMPERATURE_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  TX_THREAD *this_thread = tx_thread_identify ();
  uSWIFT_return_code_t light_ret;
  Light_Sensor light =
    { 0 };
  sbd_message_type_54_element sbd_msg_element =
    { 0 };
  light_raw_counts raw_counts;
  light_basic_counts basic_counts;
  int32_t light_thread_timeout = get_gnss_sample_window_timeout (&configuration); // Same timeout as GNSS
  struct tm time_struct =
    { 0 };
  time_t time_now = 0;

  light_sensor_init (&light, &configuration, &(light_sensor_sample_buffer[0]), &light_timer);

  tx_thread_sleep (10);

  //
  // Run tests if needed
  if ( tests.light_thread_test != NULL )
  {
    tests.light_thread_test (NULL);
  }

  if ( !light_self_test (&light) )
  {
    light_error_out (&light, LIGHT_INIT_FAILED, this_thread, "Light sensor self test failed.");
  }

  light.get_raw_measurements (&raw_counts);
  light.get_basic_counts (&basic_counts);

  LOG("Light sensor initialization complete. Raw counts:\n"
      "F1 = %hu, F2 = %hu, F3 = %hu, F4 = %hu, F5 = %hu, F6 = %hu, "
      "F7 = %hu, F8 = %hu, NIR = %hu, Clear = %hu, Dark = %hu",
      raw_counts.f1_chan, raw_counts.f2_chan, raw_counts.f3_chan, raw_counts.f4_chan,
      raw_counts.f5_chan, raw_counts.f6_chan, raw_counts.f7_chan, raw_counts.f8_chan,
      raw_counts.nir_chan, raw_counts.clear_chan, raw_counts.dark_chan);

  LOG("Basic counts:\n"
      "F1 = %u, F2 = %u, F3 = %u, F4 = %u, F5 = %u, F6 = %u, "
      "F7 = %u, F8 = %u, NIR = %u, Clear = %u, Dark = %u",
      basic_counts.f1_chan, basic_counts.f2_chan, basic_counts.f3_chan, basic_counts.f4_chan,
      basic_counts.f5_chan, basic_counts.f6_chan, basic_counts.f7_chan, basic_counts.f8_chan,
      basic_counts.nir_chan, basic_counts.clear_chan, basic_counts.dark_chan);

  (void) tx_event_flags_set (&initialization_flags, LIGHT_INIT_SUCCESS, TX_OR);

  light.idle ();
  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/

  watchdog_register_thread (LIGHT_THREAD);
  watchdog_check_in (LIGHT_THREAD);

  light.standby ();

  light.start_timer (light_thread_timeout);

  // Take our samples
  while ( 1 )
  {

    watchdog_check_in (LIGHT_THREAD);

    light_ret = light.read_all_channels ();

    if ( light_ret != uSWIFT_SUCCESS )
    {
      light_error_out (&light, LIGHT_SAMPLING_ERROR, this_thread,
                       "Error occurred when reading light sensor channels.");
    }

    if ( light_get_timeout_status () )
    {
      light_error_out (&light, LIGHT_SAMPLE_WINDOW_TIMEOUT, this_thread,
                       "Light sample window timed out after %d minutes.", light_thread_timeout);
    }

    light_ret = light.process_measurements ();

    if ( light_ret == uSWIFT_DONE_SAMPLING )
    {
      light.get_samples_averages ();
      break;
    }

    // Sleep the rest of the second away
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND - (tx_time_get () % TX_TIMER_TICKS_PER_SECOND));
  }

  light.idle ();

  light.assemble_telemetry_message_element (&sbd_msg_element);
  persistent_ram_save_message (LIGHT_TELEMETRY, (uint8_t*) &sbd_msg_element);

  watchdog_check_in (LIGHT_THREAD);

  (void) file_system_server_save_light_raw (&light);

  watchdog_check_in (LIGHT_THREAD);
  watchdog_deregister_thread (LIGHT_THREAD);

  (void) tx_event_flags_set (&complete_flags, LIGHT_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  TX_THREAD *this_thread = tx_thread_identify ();
  Turbidity_Sensor obs =
    { 0 };
  uint16_t amb, prox;
  uSWIFT_return_code_t ret;
  sbd_message_type_53_element sbd_msg_element =
    { 0 };
  int32_t turbidity_thread_timeout = get_gnss_sample_window_timeout (&configuration); // Same timeout as GNSS
  struct tm time_struct =
    { 0 };
  time_t time_now = 0;

  turbidity_sensor_init (&obs, &configuration, &turbidity_timer,
                         &turbidity_sensor_ambient_buffer[0],
                         &turbidity_sensor_proximity_buffer[0]);

  tx_thread_sleep (10);

  //
  // Run tests if needed
  if ( tests.turbidity_thread_test != NULL )
  {
    tests.turbidity_thread_test (NULL);
  }

  if ( !turbidity_self_test (&obs) )
  {
    turbidity_error_out (&obs, TURBIDITY_INIT_FAILED, this_thread,
                         "Turbidity sensor self test failed.");
  }

  obs.get_most_recent_measurement (&amb, &prox);

  LOG("Turbidity sensor initialization complete. Ambient raw counts: %d, Proximity raw counts: %d.",
      amb, prox);

  (void) tx_event_flags_set (&initialization_flags, TURBIDITY_INIT_SUCCESS, TX_OR);

  obs.idle ();
  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/

  watchdog_register_thread (TURBIDITY_THREAD);
  watchdog_check_in (TURBIDITY_THREAD);

  turbidity_reset_sample_counter ();

  gnss_get_current_lat_lon (&obs.start_lat, &obs.start_lon);

  obs.standby ();

  obs.start_timer (turbidity_thread_timeout);

  // Take our samples
  while ( 1 )
  {

    watchdog_check_in (LIGHT_THREAD);

    ret = obs.take_measurement ();

    if ( ret == uSWIFT_DONE_SAMPLING )
    {
      break;
    }
    else if ( ret != uSWIFT_SUCCESS )
    {
      turbidity_error_out (&obs, TURBIDITY_SAMPLING_ERROR, this_thread,
                           "Error occurred when reading turbidity sensor.");
    }

    if ( turbidity_get_timeout_status () )
    {
      turbidity_error_out (&obs, TURBIDITY_SAMPLING_ERROR, this_thread,
                           "Turbidity sample window timed out after %d minutes.",
                           turbidity_thread_timeout);
    }

    // Sleep the rest of the second away
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND - (tx_time_get () % TX_TIMER_TICKS_PER_SECOND));
  }

  obs.idle ();

  gnss_get_current_lat_lon (&obs.end_lat, &obs.end_lon);

  obs.assemble_telemetry_message_element (&sbd_msg_element);
  persistent_ram_save_message (TURBIDITY_TELEMETRY, (uint8_t*) &sbd_msg_element);

  watchdog_check_in (TURBIDITY_THREAD);

  (void) file_system_server_save_turbidity_raw (&obs);

  watchdog_check_in (TURBIDITY_THREAD);
  watchdog_deregister_thread (TURBIDITY_THREAD);

  (void) tx_event_flags_set (&complete_flags, TURBIDITY_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  NEDWaves_memory waves_mem =
    { 0 };

  tx_thread_sleep (1);

  if ( !waves_memory_pool_init (&waves_mem, &configuration, &(waves_byte_pool_buffer[0]),
  WAVES_MEM_POOL_SIZE) )
  {
    waves_error_out (WAVES_INIT_FAILED, this_thread, "NED Waves memory pool failed to initialize.");
  }

  (void) tx_event_flags_set (&initialization_flags, WAVES_THREAD_INIT_SUCCESS, TX_OR);
  LOG("NED Waves initialization successful.");

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

  (void) tx_event_flags_set (&complete_flags, WAVES_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
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
  Iridium iridium =
    { 0 };
  uSWIFT_return_code_t iridium_return_code = uSWIFT_SUCCESS;
  int32_t iridium_thread_timeout = configuration.iridium_max_transmit_time;
  UINT tx_return;
  char ascii_7 = '7';
  uint8_t sbd_type = 52;
  uint16_t sbd_size = 327;
  uint32_t sbd_timestamp = 0;
  uint32_t error_bits = 0;
  struct tm time_struct =
    { 0 };
  time_t time_now = 0;
  uint8_t msg_buffer[IRIDIUM_SBD_MAX_LENGTH + IRIDIUM_CHECKSUM_LENGTH] =
    { 0 };
  uint8_t *msg_ptr = (uint8_t*) &sbd_message;
  bool current_message_sent = false;
  telemetry_type_t next_message_type;
  uint32_t next_message_size = 0;

  tx_thread_sleep (10);

  if ( uart4_init () != UART_OK )
  {
    iridium_error_out (&iridium, IRIDIUM_INIT_ERROR, this_thread,
                       "Iridium UART port failed to initialize.");
  }

  iridium_init (&iridium, &configuration, device_handles.iridium_uart_handle, &iridium_uart_sema,
                device_handles.iridium_uart_tx_dma_handle,
                device_handles.iridium_uart_rx_dma_handle, &iridium_timer, &error_flags);

  iridium.on ();
  iridium.wake ();
  iridium.charge_caps ((is_first_sample_window ()) ?
      IRIDIUM_INITIAL_CAP_CHARGE_TIME : IRIDIUM_TOP_UP_CAP_CHARGE_TIME);

  if ( !iridium_apply_config (&iridium) )
  {
    iridium_error_out (&iridium, IRIDIUM_INIT_ERROR, this_thread,
                       "Iridium modem failed to initialize.");
  }

  LOG("Iridium modem initialized successfully.");
  (void) tx_event_flags_set (&initialization_flags, IRIDIUM_INIT_SUCCESS, TX_OR);

  iridium.sleep ();

  tx_thread_suspend (this_thread);

  /******************************* Control thread resumes this thread *****************************/
  watchdog_register_thread (IRIDIUM_THREAD);
  watchdog_check_in (IRIDIUM_THREAD);

  iridium.wake ();
  iridium.charge_caps (IRIDIUM_TOP_UP_CAP_CHARGE_TIME);

  iridium.start_timer (iridium_thread_timeout);

  //
  // Run tests if needed
  if ( tests.iridium_thread_test != NULL )
  {
    tests.iridium_thread_test (NULL);
  }

  // finish filling out the SBD message
  rtc_server_get_time (&time_struct, IRIDIUM_REQUEST_COMPLETE);
  time_now = mktime (&time_struct);
  sbd_timestamp = (uint32_t) time_now;
  error_bits = control_get_accumulated_error_flags ();
  memcpy (&sbd_message.legacy_number_7, &ascii_7, sizeof(char));
  memcpy (&sbd_message.type, &sbd_type, sizeof(uint8_t));
  memcpy (&sbd_message.size, &sbd_size, sizeof(uint16_t));
  memcpy (&sbd_message.timestamp, &sbd_timestamp, sizeof(float));
  memcpy (&sbd_message.error_bits, &error_bits, sizeof(uint32_t));

  msg_ptr = (uint8_t*) &sbd_message;

  if ( !iridium_apply_config (&iridium) )
  {
    // Need to save the message
    persistent_ram_save_message (WAVES_TELEMETRY, (uint8_t*) &sbd_message);
    iridium_error_out (&iridium, IRIDIUM_UART_COMMS_ERROR, this_thread,
                       "Iridium modem UART communication error.");
  }

  // copy over the message into the buffer
  memcpy (&(msg_buffer[0]), &sbd_message, sizeof(sbd_message_type_52));

  // Send the current message followed by any cached messages, until time runs out
  while ( !iridium_get_timeout_status () )
  {

    watchdog_check_in (IRIDIUM_THREAD);

    if ( !current_message_sent )
    {
      if ( iridium.transmit_message (&(msg_buffer[0]), sizeof(sbd_message_type_52))
           == uSWIFT_SUCCESS )
      {
        current_message_sent = true;
      }

      continue;
    }

    next_message_type = get_next_telemetry_message (msg_ptr, &configuration);
    if ( next_message_type == NO_MESSAGE )
    {
      break;
    }

    next_message_size = (next_message_type == WAVES_TELEMETRY) ?
        sizeof(sbd_message_type_52) :
                        (next_message_type == TURBIDITY_TELEMETRY) ?
                            sizeof(sbd_message_type_53) :
                        (next_message_type == LIGHT_TELEMETRY) ?
                            sizeof(sbd_message_type_54) : 0;

    if ( next_message_size > 0 )
    {
      if ( iridium.transmit_message (msg_ptr, next_message_size) == uSWIFT_SUCCESS )
      {
        persistent_ram_delete_message_element (next_message_type, msg_ptr);
      }
    }
  }

  watchdog_check_in (IRIDIUM_THREAD);

  if ( !current_message_sent )
  {
    // Update the error bits
    error_bits = get_current_flags (&error_flags);
    memcpy (&sbd_message.error_bits, &error_bits, sizeof(uint32_t));
    // Save the message
    persistent_ram_save_message (WAVES_TELEMETRY, msg_ptr);

    tx_thread_sleep (10);
  }

// Turn off the modem
  iridium.stop_timer ();
  iridium.sleep ();
  iridium.off ();

  iridium_deinit ();

  watchdog_check_in (IRIDIUM_THREAD);
  watchdog_deregister_thread (IRIDIUM_THREAD);

  (void) tx_event_flags_set (&complete_flags, IRIDIUM_THREAD_COMPLETED_SUCCESSFULLY, TX_OR);
  tx_thread_terminate (this_thread);
}
/* USER CODE END 1 */

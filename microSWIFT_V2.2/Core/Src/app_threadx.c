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
#include "tim.h"
#include "lpdma.h"
#include "adc.h"
#include "logger.h"

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
  HIGHEST_PRIORITY = 0,
  VERY_HIGH_PRIORITY = 1,
  HIGH_PRIORITY = 2,
  MID_PRIORITY = 3,
  LOW_PRIORITY = 4,
  VERY_LOW_PRIORITY = 5,
  LOWEST_PRIORITY = 6
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
// Used to track initialization status of threads and components
TX_EVENT_FLAGS_GROUP initialization_flags;
// We'll use flags to dictate control flow between the threads
TX_EVENT_FLAGS_GROUP thread_control_flags;
// Flags for errors
TX_EVENT_FLAGS_GROUP error_flags;
// RTC complete flags
TX_EVENT_FLAGS_GROUP rtc_complete_flags;
// Comms buses semaphores
TX_SEMAPHORE ext_rtc_spi_sema;
TX_SEMAPHORE aux_spi_1_spi_sema;
TX_SEMAPHORE aux_spi_2_spi_sema;
TX_SEMAPHORE core_i2c_sema;
TX_SEMAPHORE aux_i2c_1_sema;
TX_SEMAPHORE aux_i2c_2_sema;
TX_SEMAPHORE iridium_uart_sema;
TX_SEMAPHORE ct_uart_sema;
TX_SEMAPHORE gnss_uart_sema;
TX_SEMAPHORE aux_uart_1_sema;
TX_SEMAPHORE aux_uart_2_sema;
// Server/client message queue for RTC (including watchdog function)
TX_QUEUE rtc_messaging_queue;
TX_QUEUE logger_message_queue;
// The data structures for Waves
emxArray_real32_T *north;
emxArray_real32_T *east;
emxArray_real32_T *down;
// Messages that failed to send are stored here
Iridium_message_storage sbd_message_queue;

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
  ret = tx_byte_allocate (byte_pool, (VOID**) &pointer, M_STACK, TX_NO_WAIT);
  if ( ret != TX_SUCCESS )
  {
    return ret;
  }
  // Create the gnss thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&gnss_thread, "gnss thread", gnss_thread_entry, 0, pointer, M_STACK,
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
  // Create the waves thread. MID priority, no preemption-threshold
  ret = tx_thread_create(&waves_thread, "waves thread", waves_thread_entry, 0, pointer, XL_STACK,
                         MID_PRIORITY, HIGHEST_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
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
  // Create the event flags we'll use for triggering threads
  ret = tx_event_flags_create(&thread_control_flags, "thread flags");
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

  ret = tx_semaphore_create(&core_i2c_sema, "Core I2C sema", 0);
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

  ret = tx_semaphore_create(&gnss_uart_sema, "GNSS UART sema", 0);
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

  device_handles.core_spi_handle = &hspi1;
  device_handles.core_i2c_handle = &hi2c1;
  device_handles.iridium_uart_handle = &huart4;
  device_handles.gnss_uart_handle = &huart1;
  device_handles.ct_uart_handle = &huart5;
  device_handles.ext_flash_handle = &hospi1;
  device_handles.gnss_minutes_timer = &htim16;
  device_handles.iridium_minutes_timer = &htim17;
  device_handles.gnss_uart_rx_dma_handle = &handle_LPDMA1_Channel0;
  device_handles.battery_adc = &hadc1;
  device_handles.aux_spi_1_handle = &hspi2;
  device_handles.aux_spi_2_handle = &hspi3;
  device_handles.aux_i2c_1_handle = &hi2c2;
  device_handles.aux_i2c_2_handle = &hi2c3;
  device_handles.aux_uart_1_handle = &huart2;
  device_handles.aux_uart_2_handle = &huart3;

  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
  configuration.gnss_high_performance_mode = GNSS_HIGH_PERFORMANCE_MODE_ENABLED;
  configuration.gnss_max_acquisition_wait_time = GNSS_MAX_ACQUISITION_WAIT_TIME;
  configuration.total_ct_samples = TOTAL_CT_SAMPLES;
  configuration.windows_per_hour = SAMPLE_WINDOWS_PER_HOUR;

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

/**
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

  if ( ret != RTC_SUCCESS )
  {
    (void) tx_event_flags_set (&error_flags, RTC_ERROR, TX_OR);
    (void) tx_thread_suspend (this_thread);
    uart_logger_log_line ("RTC failed to initialize.");
  }

  // Initialize the watchdog
  ret = rtc.config_watchdog (WATCHDOG_PERIOD);

  if ( ret != RTC_SUCCESS )
  {
    shut_it_all_down ();
    // Stay stuck here
    for ( int i = 0; i < 25; i++ )
    {
      led_sequence (SELF_TEST_CRITICAL_FAULT);
    }
    HAL_NVIC_SystemReset ();
  }

  (void) tx_event_flags_set (&error_flags, RTC_INIT_SUCCESS, TX_OR);
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
        uart_logger_log_line ("RTC error detected");
        (void) tx_event_flags_set (&error_flags, RTC_ERROR, TX_OR);
        (void) tx_thread_suspend (this_thread);
      }

      *req.return_code = ret;
      (void) tx_event_flags_set (&rtc_complete_flags, req.complete_flag, TX_OR);
    }

  }
}

/**
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

/**
 * @brief  Control thread entry
 *         Primary control thread, manages all other threads and meta state
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void control_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &control_thread;
  RF_Switch rf_switch;
  Battery battery;

  self_test_status_t self_test_status = SELF_TEST_PASSED;
  ULONG actual_flags = 0;
  UINT tx_return;
  int fail_counter = 0;
  uint32_t reset_reason;

  //
  // Run tests if needed
  if ( tests.startup_test != NULL )
  {
    tests.startup_test (NULL);
  }

  // Set the watchdog reset or software reset flags
  reset_reason = HAL_RCC_GetResetSource ();

  if ( reset_reason & RCC_RESET_FLAG_PIN )
  {
    tx_event_flags_set (&error_flags, WATCHDOG_RESET, TX_OR);
  }

  if ( reset_reason & RCC_RESET_FLAG_SW )
  {
    tx_event_flags_set (&error_flags, SOFTWARE_RESET, TX_OR);
  }

  rf_switch_init (&rf_switch);

  rf_switch->power_on ();

  battery_init (&battery, device_handles.battery_adc, &thread_control_flags, &error_flags);

  // Flash some lights to let the user know its on and working
  led_sequence (INITIAL_LED_SEQUENCE);

  register_watchdog_refresh ();

  // Run the self test
  self_test_status = initial_power_on_self_test ();

  register_watchdog_refresh ();

  switch ( self_test_status )
  {
    case SELF_TEST_PASSED:
      led_sequence (TEST_PASSED_LED_SEQUENCE);
      break;

    case SELF_TEST_NON_CRITICAL_FAULT:
      led_sequence (TEST_NON_CRITICAL_FAULT_LED_SEQUENCE);
      break;

    case SELF_TEST_CRITICAL_FAULT:
      shut_it_all_down ();
      // Stay stuck here
      for ( int i = 0; i < 25; i++ )
      {
        register_watchdog_refresh ();
        led_sequence (SELF_TEST_CRITICAL_FAULT);
      }
      HAL_NVIC_SystemReset ();

    default:
      // If we got here, there's probably memory corruption
      shut_it_all_down ();
      // Stay stuck here
      while ( 1 )
      {
        register_watchdog_refresh ();
        led_sequence (SELF_TEST_CRITICAL_FAULT);
      }
  }

  // Kick off the GNSS thread
  if ( tx_thread_resume (&gnss_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

}

/**
 * @brief  Startup thread entry
 *         This thread will start all peripherals and do a systems check to
 *         make sure we're good to start the processing cycle
 * @param  ULONG thread_input - unused
 * @retval void
 */
void startup_thread_entry ( ULONG thread_input )
{
  self_test_status_t self_test_status = SELF_TEST_PASSED;
  ULONG actual_flags = 0;
  UINT tx_return;
  int fail_counter = 0;

//
// Run tests if needed
  if ( tests.startup_test != NULL )
  {
    tests.startup_test (NULL);
  }

#if WATCHDOG_ENABLED
  if ( rtc_server_config_watchdog (WATCHDOG_PERIOD, CONTROL_THREAD_REQUEST_PROCESSED)
       != RTC_SUCCESS )
  {
    self_test_status = SELF_TEST_CRITICAL_FAULT;
    return self_test_status;
  }
#endif

  // Zero out the sbd message struct
  memset (&sbd_message, 0, sizeof(sbd_message));

  // Set the watchdog reset or software reset flags
  if ( configuration.reset_reason & RCC_RESET_FLAG_PIN )
  {
    tx_event_flags_set (&error_flags, WATCHDOG_RESET, TX_OR);
  }

  if ( configuration.reset_reason & RCC_RESET_FLAG_SW )
  {
    tx_event_flags_set (&error_flags, SOFTWARE_RESET, TX_OR);
  }

  waves_memory_pool_init (&waves_byte_pool);

  if ( waves_memory_pool_create (&(waves_byte_pool_buffer[0]), WAVES_MEM_POOL_SIZE) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  // These structs are being allocated to the waves_byte_pool, and have no overlap with tx_app_byte_pool
  // Each struct has a float array written to by GNSS. These are freed after processing in waves thread.
  north = argInit_1xUnbounded_real32_T (&configuration);
  east = argInit_1xUnbounded_real32_T (&configuration);
  down = argInit_1xUnbounded_real32_T (&configuration);

  // Initialize the peripherals

  iridium_init (&iridium, &configuration, device_handles.iridium_uart_handle,
                device_handles.iridium_minutes_timer, &thread_control_flags, &error_flags,
                &sbd_message, &(iridium_error_message[0]), &(iridium_response_message[0]),
                &sbd_message_queue);

#if TEMPERATURE_ENABLED
  temperature_init (&temperature, device_handles.core_i2c_handle, &thread_control_flags,
                    &error_flags, TEMP_FET_GPIO_Port, TEMP_FET_Pin, true);
#endif

  rf_switch_init (&rf_switch);

  battery_init (&battery, device_handles.battery_adc, &thread_control_flags, &error_flags);

  tx_return = tx_event_flags_get (&thread_control_flags, FULL_CYCLE_COMPLETE, TX_OR_CLEAR,
                                  &actual_flags, TX_NO_WAIT);
// If this is a subsequent window, just setup the GNSS, skip the rest
  if ( tx_return == TX_SUCCESS )
  {

    // Increment the sample window counter
    sample_window_counter++;

    register_watchdog_refresh ();

    HAL_GPIO_WritePin (GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);

    // Reset all threads
    tx_return = tx_thread_reset (&gnss_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&gnss_thread);
      tx_thread_reset (&gnss_thread);
    }
#if CT_ENABLED
    tx_return = tx_thread_reset (&ct_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&ct_thread);
      tx_thread_reset (&ct_thread);
    }
#endif

#if TEMPERATURE_ENABLED
    tx_return = tx_thread_reset (&temperature_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&temperature_thread);
      tx_thread_reset (&temperature_thread);
    }
#endif
    tx_return = tx_thread_reset (&waves_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&waves_thread);
      tx_thread_reset (&waves_thread);
    }
    tx_return = tx_thread_reset (&iridium_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&iridium_thread);
      tx_thread_reset (&iridium_thread);
    }
    tx_return = tx_thread_reset (&end_of_cycle_thread);
    if ( tx_return == TX_NOT_DONE )
    {
      tx_thread_terminate (&end_of_cycle_thread);
      tx_thread_reset (&end_of_cycle_thread);
    }

    // Power up the RF switch
    rf_switch->power_on ();

    // Check if there was a GNSS error. If so, reconfigure device
    tx_return = tx_event_flags_get (&thread_control_flags, GNSS_CONFIG_REQUIRED,
    TX_OR_CLEAR,
                                    &actual_flags, TX_NO_WAIT);
    if ( tx_return == TX_SUCCESS )
    {
      fail_counter = 0;
      while ( fail_counter < MAX_SELF_TEST_RETRIES )
      {

        register_watchdog_refresh ();

        if ( gnss->config () != GNSS_SUCCESS )
        {
          // Config didn't work, cycle power and try again
          gnss->cycle_power ();
          fail_counter++;
        }
        else
        {
          break;
        }
      }
    }
    // If we couldn't configure the GNSS, send a reset vector
    if ( fail_counter == MAX_SELF_TEST_RETRIES )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }

    // Kick off the GNSS thread
    if ( tx_thread_resume (&gnss_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }
  }

// This is first time power up, test everything and flash LED sequence
  else
  {
    register_watchdog_refresh ();
    // Flash some lights to let the user know its on and working
    led_sequence (INITIAL_LED_SEQUENCE);

    rf_switch->power_on ();

    register_watchdog_refresh ();

    self_test_status = initial_power_on_self_test ();

    register_watchdog_refresh ();

    switch ( self_test_status )
    {
      case SELF_TEST_PASSED:
        led_sequence (TEST_PASSED_LED_SEQUENCE);
        break;

      case SELF_TEST_NON_CRITICAL_FAULT:
        led_sequence (TEST_NON_CRITICAL_FAULT_LED_SEQUENCE);
        break;

      case SELF_TEST_CRITICAL_FAULT:
        shut_it_all_down ();
        // Stay stuck here
        for ( int i = 0; i < 25; i++ )
        {
          register_watchdog_refresh ();
          led_sequence (SELF_TEST_CRITICAL_FAULT);
        }
        HAL_NVIC_SystemReset ();

      default:
        // If we got here, there's probably a memory corruption
        shut_it_all_down ();
        // Stay stuck here
        while ( 1 )
        {
          register_watchdog_refresh ();
          led_sequence (SELF_TEST_CRITICAL_FAULT);
        }
    }

    register_watchdog_refresh ();
    // Kick off the GNSS thread
    if ( tx_thread_resume (&gnss_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }
  }

// We're done, terminate this thread
  tx_thread_terminate (&startup_thread);
}

/**
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
  uint8_t ubx_DMA_message_buf[UBX_MESSAGE_SIZE * 2];
  uint8_t ubx_message_process_buf[UBX_MESSAGE_SIZE * 2];
  uint8_t gnss_config_response_buf[GNSS_CONFIG_BUFFER_SIZE];

  gnss_error_code_t gnss_return_code;
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

  gnss_init (&gnss, &configuration, device_handles.gnss_uart_handle,
             device_handles.gnss_uart_rx_dma_handle, &thread_control_flags, &error_flags,
             device_handles.gnss_minutes_timer, &(ubx_message_process_buf[0]),
             &(gnss_config_response_buf[0]), north->data, east->data, down->data);

  register_watchdog_refresh ();

//
// Run tests if needed
  if ( tests.gnss_thread_test != NULL )
  {
    tests.gnss_thread_test (NULL);
  }

  register_watchdog_refresh ();

// Calculate the max acq time if we're running multiple windows per hour
  if ( configuration.windows_per_hour > 1 )
  {

    // If its the first sample widnow after a power on, give more time, else
    // calculate the amount of time
    if ( sample_window_counter == 0 )
    {

      gnss_max_acq_time = configuration.gnss_max_acquisition_wait_time;

    }
    else
    {
      // Calculate how much time we have to get a fix before we're overlapping
      // with the next window
      gnss_max_acq_time = (60 / configuration.windows_per_hour)
                          - configuration.iridium_max_transmit_time - sample_window_timeout;
    }
  }
  else
  {

    // Only 1 sample window per hour
    gnss_max_acq_time = configuration.gnss_max_acquisition_wait_time;

  }

// Ensure we have valid settings such that gnss_max_acq_time if a positive value
  while ( 1 >= gnss_max_acq_time )
  {
    // The watchdog will initiate a restart after it's window runs out
    led_sequence (SELF_TEST_CRITICAL_FAULT);
  }

  register_watchdog_refresh ();

// Grab the RF switch
  rf_switch->set_gnss_port ();

// Start the timer for resolution stages
  HAL_TIM_Base_Stop_IT (gnss->minutes_timer);
  gnss->reset_timer (configuration.gnss_max_acquisition_wait_time);
  HAL_TIM_Base_Start_IT (gnss->minutes_timer);

// Wait until we get a series of good UBX_NAV_PVT messages and are
// tracking a good number of satellites before moving on
  if ( gnss->sync_and_start_reception (start_GNSS_UART_DMA, ubx_DMA_message_buf,
  UBX_MESSAGE_SIZE)
       != GNSS_SUCCESS )
  {
    // If we were unable to get good GNSS reception and start the DMA transfer loop, then
    // go to sleep until the top of the next hour. Sleep will be handled in end_of_cycle_thread
    register_watchdog_refresh ();
    jump_to_end_of_window (GNSS_FRAME_SYNC_ERROR);
  }
  else
  {
    gnss->is_configured = true;
  }

  while ( !(gnss->all_resolution_stages_complete || gnss->timer_timeout) )
  {

    register_watchdog_refresh ();
    tx_return = tx_event_flags_get (&thread_control_flags,
                                    ((GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE)),
                                    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {
      gnss->process_message ();
    }
    // Message was dropped or incomplete
    else if ( (tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      continue;

    }
    // Any other error code is indication of memory corruption
    else
    {

      register_watchdog_refresh ();
      jump_to_end_of_window (MEMORY_CORRUPTION_ERROR);
    }

  }

// If this evaluates to true, we were unable to get adequate GNSS reception to
// resolve time and get at least 1 good sample. Go to sleep until the top of the next hour.
// Sleep will be handled in end_of_cycle_thread
  if ( gnss->timer_timeout )
  {
    register_watchdog_refresh ();
    jump_to_end_of_window (GNSS_RESOLUTION_ERROR);
  }

// We were able to resolve time within the given window of time. Now start the timer to ensure
// the sample window doesn't take too long
  HAL_TIM_Base_Stop_IT (gnss->minutes_timer);
  gnss->reset_timer (sample_window_timeout);
  HAL_TIM_Base_Start_IT (gnss->minutes_timer);

// Wait until all the samples have been processed
  while ( !gnss->all_samples_processed )
  {

    register_watchdog_refresh ();

    tx_return = tx_event_flags_get (&thread_control_flags, GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE,
    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      gnss->process_message ();
      number_of_no_sample_errors = 0;

    }
    // Message was dropped or incomplete
    else if ( (tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      gnss_return_code = gnss->get_running_average_velocities ();

      if ( gnss_return_code == GNSS_NO_SAMPLES_ERROR )
      {
        if ( ++number_of_no_sample_errors == configuration.gnss_sampling_rate * 60 )
        {
          register_watchdog_refresh ();
          jump_to_end_of_window (SAMPLE_WINDOW_ERROR);
        }

      }

    }
    // Any other error code is indication of memory corruption
    else
    {

      register_watchdog_refresh ();
      jump_to_end_of_window (MEMORY_CORRUPTION_ERROR);
    }

    // If this evaluates to true, something hung up with GNSS sampling. End the sample window
    if ( gnss->timer_timeout )
    {
      register_watchdog_refresh ();
      jump_to_end_of_window (SAMPLE_WINDOW_ERROR);
    }
  }

  register_watchdog_refresh ();

// Stop the timer
  HAL_TIM_Base_Stop_IT (gnss->minutes_timer);
// turn off the GNSS sensor
  gnss->on_off (GPIO_PIN_RESET);
// Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (gnss->gnss_uart_handle);
  HAL_DMA_DeInit (gnss->gnss_rx_dma_handle);
  HAL_DMA_DeInit (gnss->gnss_tx_dma_handle);

  gnss->get_location (&last_lat, &last_lon);
// Just to be overly sure about alignment
  memcpy (&sbd_message.Lat, &last_lat, sizeof(float));
  memcpy (&sbd_message.Lon, &last_lon, sizeof(float));
// We're using the "port" field to encode how many samples were averaged divided by 10,
// up to the limit of an uint8_t
  sbd_port = ((gnss->total_samples_averaged / 10) >= 255) ?
      255 : (gnss->total_samples_averaged / 10);
  memcpy (&sbd_message.port, &sbd_port, sizeof(uint8_t));

// Port the RF switch to the modem
  rf_switch->set_iridium_port ();

  register_watchdog_refresh ();

#if CT_ENABLED

  if ( tx_thread_resume (&ct_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

#elif TEMPERATURE_ENABLED

  if ( tx_thread_resume (&temperature_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

#else

    if ( tx_thread_resume (&waves_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }

  #endif

  tx_thread_terminate (&gnss_thread);
}

/**
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
  CHAR ct_data[CT_DATA_ARRAY_SIZE];
  ct_samples ct_samples_buf[TOTAL_CT_SAMPLES];

  ct_error_code_t ct_return_code;
  uint32_t ct_parsing_error_counter;
  real16_T half_salinity;
  real16_T half_temp;
  int fail_counter;

  ct_init (&ct, &configuration, device_handles.ct_uart_handle, &thread_control_flags, &error_flags,
           &(ct_data[0]), &(ct_samples_buf[0]));

  register_watchdog_refresh ();

  //
  // Run tests if needed
  if ( tests.ct_thread_test != NULL )
  {
    tests.ct_thread_test (NULL);
  }

  register_watchdog_refresh ();

  // Set the mean salinity and temp values to error values in the event the sensor fails
  half_salinity.bitPattern = CT_AVERAGED_VALUE_ERROR_CODE;
  half_temp.bitPattern = CT_AVERAGED_VALUE_ERROR_CODE;

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  // The first message will have a different frame length from a header, so adjust the fail
  // counter appropriately
  fail_counter = -1;
  // Turn on the CT sensor, warm it up, and frame sync
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();

    ct_return_code = ct->self_test (false);
    if ( ct_return_code != CT_SUCCESS )
    {
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;

    }
    else
    {

      break;
    }
  }

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();
    jump_to_waves ();
  }

  // Take our samples
  ct_parsing_error_counter = 0;
  while ( ct->total_samples < configuration.total_ct_samples )
  {

    register_watchdog_refresh ();
    ct_return_code = ct->parse_sample ();

    if ( ct_return_code == CT_PARSING_ERROR )
    {
      ct_parsing_error_counter++;
    }

    if ( (ct_parsing_error_counter >= 10) || (ct_return_code == CT_UART_ERROR) )
    {
      // If there are too many parsing errors or a UART error occurs, then
      // stop trying and
      register_watchdog_refresh ();
      jump_to_waves ();
    }

    if ( ct_return_code == CT_DONE_SAMPLING )
    {
      break;
    }
  }

  register_watchdog_refresh ();

  // Turn off the CT sensor
  ct->on_off (GPIO_PIN_RESET);
  // Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (ct->ct_uart_handle);
  HAL_DMA_DeInit (ct->ct_dma_handle);

  // Got our samples, now average them
  ct_return_code = ct->get_averages ();
  // Make sure something didn't go terribly wrong
  if ( ct_return_code == CT_NOT_ENOUGH_SAMPLES )
  {
    register_watchdog_refresh ();
    jump_to_waves ();
  }

  // Now set the mean salinity and temp values to the real ones
  half_salinity = floatToHalf ((float) ct->averages.salinity);
  half_temp = floatToHalf ((float) ct->averages.temp);

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  register_watchdog_refresh ();

  if ( tx_thread_resume (&waves_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_thread_terminate (&ct_thread);
}

/**
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

  temperature_error_code_t temp_return_code;
  real16_T half_salinity;
  real16_T half_temp;
  int fail_counter = 0;

  temperature_init (&temperature, device_handles.core_i2c_handle, &thread_control_flags,
                    &error_flags, TEMP_FET_GPIO_Port, TEMP_FET_Pin, true);

  register_watchdog_refresh ();

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  register_watchdog_refresh ();

  temperature->on ();

  while ( fail_counter < MAX_RETRIES )
  {
    temp_return_code = temperature->get_readings ();

    if ( temp_return_code == TEMPERATURE_SUCCESS )
    {
      break;
    }
    else
    {
      fail_counter++;
    }
  }

  if ( fail_counter == MAX_RETRIES )
  {
    half_temp.bitPattern = TEMPERATURE_AVERAGED_ERROR_CODE;
  }
  else
  {
    half_temp = floatToHalf (temperature->converted_temp);
  }

  temperature->off ();

  half_salinity.bitPattern = CT_AVERAGED_VALUE_ERROR_CODE;
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));
  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));

  if ( tx_thread_resume (&waves_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }
  tx_thread_terminate (&temperature_thread);
}

/**
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

}

/**
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

}

/**
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
}

/**
 * @brief  waves_thread_entry
 *         This thread will run the GPSWaves algorithm.
 *
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void waves_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  TX_THREAD *this_thread = &waves_thread;

  waves_memory_pool_init (&waves_byte_pool);

  if ( waves_memory_pool_create (&(waves_byte_pool_buffer[0]), WAVES_MEM_POOL_SIZE) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  // These structs are being allocated to the waves_byte_pool, and have no overlap with tx_app_byte_pool
  // Each struct has a float array written to by GNSS. These are freed after processing in waves thread.
  north = argInit_1xUnbounded_real32_T (&configuration);
  east = argInit_1xUnbounded_real32_T (&configuration);
  down = argInit_1xUnbounded_real32_T (&configuration);

  register_watchdog_refresh ();

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  register_watchdog_refresh ();

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
  NEDwaves_memlight (north, east, down, gnss->sample_window_freq, &Hs, &Tp, &Dp, E, &b_fmin,
                     &b_fmax, a1, b1, a2, b2, check);

  emxDestroyArray_real32_T (down);
  emxDestroyArray_real32_T (east);
  emxDestroyArray_real32_T (north);

// Delete the memory pool to fix the memory leak in NEDwaves_memlight
  if ( waves_memory_pool_delete () != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

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

  register_watchdog_refresh ();

  if ( tx_thread_resume (&iridium_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_thread_terminate (&waves_thread);
}

/**
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
  uint8_t iridium_response_message[IRIDIUM_MAX_RESPONSE_SIZE];
  uint8_t iridium_error_message[IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH + 64];

  iridium_error_code_t iridium_return_code;
  ULONG actual_error_flags = 0;
  ULONG error_occured_flags = GNSS_ERROR | MODEM_ERROR | MEMORY_ALLOC_ERROR | DMA_ERROR | UART_ERROR
                              | RTC_ERROR | WATCHDOG_RESET | SOFTWARE_RESET | GNSS_RESOLUTION_ERROR;

  iridium_init (&iridium, &configuration, device_handles.iridium_uart_handle,
                device_handles.iridium_minutes_timer, &thread_control_flags, &error_flags,
                &sbd_message, &(iridium_error_message[0]), &(iridium_response_message[0]),
                &sbd_message_queue);

  UINT tx_return;
  int fail_counter;
  char ascii_7 = '7';
  uint8_t sbd_type = 52;
  uint16_t sbd_size = 327;
  real16_T voltage;
  float sbd_timestamp = iridium->get_timestamp ();
  bool queue_empty = iridium->storage_queue->num_msgs_enqueued == 0;

  register_watchdog_refresh ();

  //
  // Run tests if needed
  if ( tests.iridium_thread_test != NULL )
  {
    tests.iridium_thread_test (NULL);
  }

  register_watchdog_refresh ();

// Check if we are skipping this message
  tx_return = tx_event_flags_get (&error_flags, GNSS_EXITED_EARLY, TX_OR_CLEAR, &actual_error_flags,
  TX_NO_WAIT);

  if ( tx_return == TX_SUCCESS )
  {
    iridium->skip_current_message = true;
  }

// If this message was skipped and there's nothing in the queue, exit and jump to end_of_cycle_thread
  if ( iridium->skip_current_message && queue_empty )
  {
    // Turn off the modem and RF switch
    iridium->sleep (GPIO_PIN_RESET);
    iridium->on_off (GPIO_PIN_RESET);
    rf_switch->power_off ();
    // Deinit UART and DMA to prevent spurious interrupts
    HAL_UART_DeInit (iridium->iridium_uart_handle);
    HAL_DMA_DeInit (iridium->iridium_rx_dma_handle);
    HAL_DMA_DeInit (iridium->iridium_tx_dma_handle);
    HAL_TIM_Base_Stop_IT (iridium->timer);

    // Resume EOC thread
    if ( tx_thread_resume (&end_of_cycle_thread) != TX_SUCCESS )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }

    tx_thread_terminate (&iridium_thread);
  }

// Port the RF switch to the modem
  rf_switch->set_iridium_port ();

// Grab the battery voltage
  battery->start_conversion ();
  battery->get_voltage (&voltage);
  battery->shutdown_adc ();
  memcpy (&sbd_message.mean_voltage, &voltage, sizeof(real16_T));

// finish filling out the SBD message
  memcpy (&sbd_message.legacy_number_7, &ascii_7, sizeof(char));
  memcpy (&sbd_message.type, &sbd_type, sizeof(uint8_t));
  memcpy (&sbd_message.size, &sbd_size, sizeof(uint16_t));
  memcpy (&sbd_message.timestamp, &sbd_timestamp, sizeof(float));

// Last posit was placed in the SBD message in GNSS thread, copy that over to
// the Iridium struct
  memcpy (&iridium->current_lat, &sbd_message.Lat, sizeof(float));
  memcpy (&iridium->current_lon, &sbd_message.Lon, sizeof(float));

// This will turn on the modem and make sure the caps are charged
  iridium->charge_caps (IRIDIUM_TOP_UP_CAP_CHARGE_TIME);

  fail_counter = 0;
  while ( fail_counter < MAX_SELF_TEST_RETRIES )
  {

    register_watchdog_refresh ();
    // See if we can get an ack message from the modem
    iridium_return_code = iridium->self_test ();
    if ( iridium_return_code != IRIDIUM_SUCCESS )
    {

      iridium->cycle_power ();
      tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
      fail_counter++;

    }
    else
    {

      break;
    }
  }

  if ( fail_counter == MAX_SELF_TEST_RETRIES )
  {
    // If we couldn't get a response from the modem, shut everything down and force reset.
    // But save the current message first
    if ( !iridium->skip_current_message )
    {
      iridium->queue_add (iridium->current_message);
    }
    shut_it_all_down ();
    tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);
    HAL_NVIC_SystemReset ();
  }

  register_watchdog_refresh ();

  iridium->transmit_message ();

  register_watchdog_refresh ();

// Check for error messages
  tx_event_flags_get (&error_flags, error_occured_flags, TX_OR_CLEAR, &actual_error_flags,
  TX_NO_WAIT);

// If we have an error flag, send an error message
  if ( actual_error_flags )
  {
    register_watchdog_refresh ();
    send_error_message (actual_error_flags);

    register_watchdog_refresh ();
  }

// If something went wrong with the RTC, we'll reset
  if ( actual_error_flags & RTC_ERROR )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

// Turn off the modem and RF switch
  iridium->sleep (GPIO_PIN_RESET);
  iridium->on_off (GPIO_PIN_RESET);
  rf_switch->power_off ();
// Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (iridium->iridium_uart_handle);
  HAL_DMA_DeInit (iridium->iridium_rx_dma_handle);
  HAL_DMA_DeInit (iridium->iridium_tx_dma_handle);
  HAL_TIM_Base_Stop_IT (iridium->timer);

  if ( tx_thread_resume (&end_of_cycle_thread) != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_thread_terminate (&iridium_thread);
}

/**
 * @brief  teardown_thread_entry
 *         This thread will execute when either an error flag is set or all
 *         the done flags are set, indicating we are ready to shutdown until
 *         the next window.
 * @param  ULONG thread_input - unused
 * @retval void
 */
static void end_of_cycle_thread_entry ( ULONG thread_input )
{
  UNUSED(thread_input);
  RTC_AlarmTypeDef alarm =
    { 0 };
  RTC_TimeTypeDef initial_rtc_time;
  RTC_TimeTypeDef rtc_time;
  RTC_DateTypeDef rtc_date;
  int32_t wake_up_minute;
  UINT tx_return;

// Must put this thread to sleep for a short while to allow other threads to terminate
  tx_thread_sleep (1);

  register_watchdog_refresh ();

  //
  // Run tests if needed
  if ( tests.shutdown_test != NULL )
  {
    tests.shutdown_test (NULL);
  }

  register_watchdog_refresh ();

// Just to be overly sure everything is off
  shut_it_all_down ();
  tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND / 10);

//      Clear any pending interrupts, See Errata section 2.2.4

#if CT_ENABLED
  HAL_NVIC_DisableIRQ (UART4_IRQn);
  HAL_NVIC_ClearPendingIRQ (UART4_IRQn);
#endif

  HAL_NVIC_DisableIRQ (GPDMA1_Channel0_IRQn);
  HAL_NVIC_DisableIRQ (GPDMA1_Channel1_IRQn);
  HAL_NVIC_DisableIRQ (GPDMA1_Channel2_IRQn);
  HAL_NVIC_DisableIRQ (GPDMA1_Channel3_IRQn);
  HAL_NVIC_DisableIRQ (GPDMA1_Channel4_IRQn);
  HAL_NVIC_DisableIRQ (UART5_IRQn);
  HAL_NVIC_DisableIRQ (LPUART1_IRQn);

  HAL_NVIC_ClearPendingIRQ (RTC_S_IRQn);
  HAL_NVIC_ClearPendingIRQ (GPDMA1_Channel0_IRQn);
  HAL_NVIC_ClearPendingIRQ (GPDMA1_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ (GPDMA1_Channel2_IRQn);
  HAL_NVIC_ClearPendingIRQ (GPDMA1_Channel3_IRQn);
  HAL_NVIC_ClearPendingIRQ (GPDMA1_Channel4_IRQn);
  HAL_NVIC_ClearPendingIRQ (UART5_IRQn);
  HAL_NVIC_ClearPendingIRQ (LPUART1_IRQn);

  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG (hrtc, RTC_CLEAR_WUTF);
  __HAL_RTC_ALARM_CLEAR_FLAG (hrtc, RTC_CLEAR_ALRAF);
  HAL_NVIC_ClearPendingIRQ (RTC_IRQn);

//      // Only used for low power modes lower than stop2.
//      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

  HAL_RTC_GetTime (device_handles.hrtc, &initial_rtc_time, RTC_FORMAT_BIN);
// Must call GetDate to keep the RTC happy, even if you don't use it
  HAL_RTC_GetDate (device_handles.hrtc, &rtc_date, RTC_FORMAT_BIN);

#ifdef SHORT_SLEEP
        wake_up_minute = initial_rtc_time.Minutes >= 59 ? (initial_rtc_time.Minutes + 1) - 60 :
                        (initial_rtc_time.Minutes + 1);
#else

// Handle depending on how many sample windows we're doing per hour
// Handle the simple case first
  if ( configuration.windows_per_hour == 1 )
  {
    wake_up_minute = 0;
  }
  else
  {
    wake_up_minute = 60 / configuration.windows_per_hour;

    // Keep adding the quantity determined above until the result is positive
    while ( (wake_up_minute - initial_rtc_time.Minutes) <= 0 )
    {
      wake_up_minute += (60 / configuration.windows_per_hour);
    }

    // Make sure it's a valid minute!!
    wake_up_minute %= 60;
  }

#endif

  HAL_GPIO_WritePin (GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);

  while ( rtc_time.Minutes != wake_up_minute )
  {

    // Get the date and time
    HAL_RTC_GetTime (device_handles.hrtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate (device_handles.hrtc, &rtc_date, RTC_FORMAT_BIN);

    // We should be restarting the window at the top of the hour. If the initial time and just
    // checked time differ in hours, then we should start a new window. This should never occur,
    // but just as a second safety
    if ( initial_rtc_time.Hours != rtc_time.Hours )
    {
      SystemClock_Config ();
      register_watchdog_refresh ();
      HAL_ResumeTick ();
      HAL_ICACHE_Enable ();
      HAL_PWREx_DisableRAMsContentStopRetention (PWR_SRAM4_FULL_STOP_RETENTION);
      HAL_PWREx_DisableRAMsContentStopRetention (PWR_ICACHE_FULL_STOP_RETENTION);
      HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM1_FULL_STOP_RETENTION);
      HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM2_FULL_STOP_RETENTION);
      HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM3_FULL_STOP_RETENTION);
      break;
    }

    // Set the alarm to wake up the processor in 25 seconds
    alarm.Alarm = RTC_ALARM_A;
    alarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    alarm.AlarmTime = rtc_time;
    alarm.AlarmTime.Seconds = (rtc_time.Seconds >= 35) ?
        ((rtc_time.Seconds + 25) - 60) : (rtc_time.Seconds + 25);
    alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
    alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;

    // If something goes wrong setting the alarm, force an RTC reset and go to the next window.
    // With luck, the RTC will get set again on the next window and everything will be cool.
    if ( HAL_RTC_SetAlarm_IT (device_handles.hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK )
    {
      shut_it_all_down ();
      HAL_NVIC_SystemReset ();
    }

    register_watchdog_refresh ();
    // See errata regarding ICACHE access on wakeup, section 2.2.11
    HAL_ICACHE_Disable ();
    HAL_SuspendTick ();

    HAL_PWREx_EnterSTOP2Mode (PWR_STOPENTRY_WFI);

    register_watchdog_refresh ();
    // Restore clocks to the same config as before stop2 mode
    SystemClock_Config ();
    HAL_ResumeTick ();
    HAL_ICACHE_Enable ();
    HAL_PWREx_DisableRAMsContentStopRetention (PWR_SRAM4_FULL_STOP_RETENTION);
    HAL_PWREx_DisableRAMsContentStopRetention (PWR_ICACHE_FULL_STOP_RETENTION);
    HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM1_FULL_STOP_RETENTION);
    HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM2_FULL_STOP_RETENTION);
    HAL_PWREx_EnableRAMsContentStopRetention (PWR_SRAM3_FULL_STOP_RETENTION);

  }

  register_watchdog_refresh ();

// Disable the RTC Alarm and clear flags
  HAL_RTC_DeactivateAlarm (device_handles.hrtc, RTC_ALARM_A);
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG (hrtc, RTC_CLEAR_WUTF);
  __HAL_RTC_ALARM_CLEAR_FLAG (hrtc, RTC_CLEAR_ALRAF);
  HAL_NVIC_ClearPendingIRQ (RTC_IRQn);

#if CT_ENABLED
  HAL_NVIC_EnableIRQ (UART4_IRQn);
#endif
  HAL_NVIC_EnableIRQ (GPDMA1_Channel0_IRQn);
  HAL_NVIC_EnableIRQ (GPDMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ (GPDMA1_Channel2_IRQn);
  HAL_NVIC_EnableIRQ (GPDMA1_Channel3_IRQn);
  HAL_NVIC_EnableIRQ (GPDMA1_Channel4_IRQn);
  HAL_NVIC_EnableIRQ (UART5_IRQn);
  HAL_NVIC_EnableIRQ (LPUART1_IRQn);
//      HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

// Reset and resume the startup thread
  tx_return = tx_thread_reset (&startup_thread);
  if ( tx_return != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_return = tx_thread_resume (&startup_thread);
  if ( tx_return != TX_SUCCESS )
  {
    shut_it_all_down ();
    HAL_NVIC_SystemReset ();
  }

  tx_event_flags_set (&thread_control_flags, FULL_CYCLE_COMPLETE, TX_OR);
  tx_thread_terminate (&end_of_cycle_thread);
}
/* USER CODE END 1 */

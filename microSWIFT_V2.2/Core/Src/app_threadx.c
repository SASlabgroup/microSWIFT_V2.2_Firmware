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
#include "threadx_support.h"
#include "watchdog.h"

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

  watchdog_check_in ();

  // Run the self test
  self_test_status = initial_power_on_self_test ();

  watchdog_check_in ();

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
        watchdog_check_in ();
        led_sequence (SELF_TEST_CRITICAL_FAULT);
      }
      HAL_NVIC_SystemReset ();

    default:
      // If we got here, there's probably memory corruption
      shut_it_all_down ();
      // Stay stuck here
      while ( 1 )
      {
        watchdog_check_in ();
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
  float *north, *east, *down;

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

  // Make sure the waves thread has initialized properly before proceeding
  while ( 1 )
  {
    if ( waves_memory_get_raw_data_pointers (north, east, down) )
    {
      break;
    }
    tx_thread_sleep (1);
  }

  gnss_init (&gnss, &configuration, device_handles.gnss_uart_handle,
             device_handles.gnss_uart_rx_dma_handle, &thread_control_flags, &error_flags,
             device_handles.gnss_minutes_timer, &(ubx_message_process_buf[0]),
             &(gnss_config_response_buf[0]), north, east, down);

  if ( !gnss_apply_config (&gnss) )
  {
    uart_logger_log_line ("GNSS failed to initialize.");
    tx_thread_suspend (this_thread);
  }

  (void) tx_event_flags_set (&initialization_flags, GNSS_INIT_SUCCESS, TX_OR);
  uart_logger_log_line ("GNSS initialization successful.");

  watchdog_register_thread (GNSS_THREAD);
  watchdog_check_in (GNSS_THREAD);

  //
  // Run tests if needed
  if ( tests.gnss_thread_test != NULL )
  {
    tests.gnss_thread_test (NULL);
  }

  // Calculate the max acquisition time if we're running multiple windows per hour
  if ( configuration.windows_per_hour > 1 )
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
    gnss_max_acq_time = configuration.gnss_max_acquisition_wait_time;
  }

  if ( gnss_max_acq_time <= 1 )
  {
    uart_logger_log_line ("Invalid GNSS max acquisition time.");
    (void) tx_event_flags_set (&error_flags, INVALID_CONFIGURATION, TX_OR);
    tx_thread_suspend (this_thread);
  }

  watchdog_check_in ();

  // Start the timer for resolution stages
  HAL_TIM_Base_Stop_IT (gnss.minutes_timer);
  gnss.reset_timer (configuration.gnss_max_acquisition_wait_time);
  HAL_TIM_Base_Start_IT (gnss.minutes_timer);

  // Wait until we get a series of good UBX_NAV_PVT messages and are
  // tracking a good number of satellites before moving on
  if ( gnss.sync_and_start_reception (start_GNSS_UART_DMA, ubx_DMA_message_buf,
  UBX_MESSAGE_SIZE)
       != GNSS_SUCCESS )
  {
    // If we were unable to get good GNSS reception and start the DMA transfer loop, then
    // go to sleep until the top of the next hour. Sleep will be handled in end_of_cycle_thread
    watchdog_check_in ();
    jump_to_end_of_window (GNSS_RESOLUTION_ERROR);
  }
  else
  {
    gnss.is_configured = true;
    uart_logger_log_line ("GNSS switching to circular DMA mode.");
  }

  while ( !(gnss.all_resolution_stages_complete || gnss.timer_timeout) )
  {

    watchdog_check_in ();
    tx_return = tx_event_flags_get (&thread_control_flags,
                                    ((GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE)),
                                    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {
      gnss.process_message ();
    }
    // Message was dropped or incomplete
    else if ( (tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      continue;

    }
    // Any other error code is indication of memory corruption
    else
    {

      watchdog_check_in ();
      jump_to_end_of_window (MEMORY_CORRUPTION_ERROR);
    }

  }

  // If this evaluates to true, we were unable to get adequate GNSS reception to
  // resolve time and get at least 1 good sample. Go to sleep until the top of the next hour.
  // Sleep will be handled in end_of_cycle_thread
  if ( gnss.timer_timeout )
  {
    watchdog_check_in ();
    jump_to_end_of_window (GNSS_RESOLUTION_ERROR);
  }

  // We were able to resolve time within the given window of time. Now start the timer to ensure
  // the sample window doesn't take too long
  HAL_TIM_Base_Stop_IT (gnss.minutes_timer);
  gnss.reset_timer (sample_window_timeout);
  HAL_TIM_Base_Start_IT (gnss.minutes_timer);

  // Wait until all the samples have been processed
  while ( !gnss.all_samples_processed )
  {

    watchdog_check_in ();

    tx_return = tx_event_flags_get (&thread_control_flags, GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE,
    TX_OR_CLEAR,
                                    &actual_flags, timer_ticks_to_get_message);

    // Full message came through
    if ( (tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      gnss.process_message ();
      number_of_no_sample_errors = 0;

    }
    // Message was dropped or incomplete
    else if ( (tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE) )
    {

      gnss_return_code = gnss.get_running_average_velocities ();

      if ( gnss_return_code == GNSS_NO_SAMPLES_ERROR )
      {
        if ( ++number_of_no_sample_errors == configuration.gnss_sampling_rate * 60 )
        {
          watchdog_check_in ();
          jump_to_end_of_window (SAMPLE_WINDOW_ERROR);
        }

      }

    }
    // Any other error code is indication of memory corruption
    else
    {

      watchdog_check_in ();
      jump_to_end_of_window (MEMORY_CORRUPTION_ERROR);
    }

    // If this evaluates to true, something hung up with GNSS sampling. End the sample window
    if ( gnss.timer_timeout )
    {
      watchdog_check_in ();
      jump_to_end_of_window (SAMPLE_WINDOW_ERROR);
    }
  }

  watchdog_check_in ();

  // Stop the timer
  HAL_TIM_Base_Stop_IT (gnss.minutes_timer);
  // turn off the GNSS sensor
  gnss.on_off (GPIO_PIN_RESET);
  // Deinit UART and DMA to prevent spurious interrupts
  HAL_UART_DeInit (gnss.gnss_uart_handle);
  HAL_DMA_DeInit (gnss.gnss_rx_dma_handle);
  HAL_DMA_DeInit (gnss.gnss_tx_dma_handle);

  gnss.get_location (&last_lat, &last_lon);
  // Just to be overly sure about alignment
  memcpy (&sbd_message.Lat, &last_lat, sizeof(float));
  memcpy (&sbd_message.Lon, &last_lon, sizeof(float));
  // We're using the "port" field to encode how many samples were averaged divided by 10,
  // up to the limit of an uint8_t
  sbd_port = ((gnss.total_samples_averaged / 10) >= 255) ?
      255 : (gnss.total_samples_averaged / 10);
  memcpy (&sbd_message.port, &sbd_port, sizeof(uint8_t));

  watchdog_check_in ();

  uart_logger_log_line ("GNSS Thread complete, now terminating.");
  (void) tx_event_flags_set (&control_flags, GNSS_DONE, TX_OR);
  tx_thread_terminate (this_thread);
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
  ct_samples self_test_readings =
    { 0 };

  ct_error_code_t ct_return_code;
  uint32_t ct_parsing_error_counter;
  real16_T half_salinity;
  real16_T half_temp;
  int fail_counter;

  ct_init (&ct, &configuration, device_handles.ct_uart_handle, &thread_control_flags, &error_flags,
           &(ct_data[0]), &(ct_samples_buf[0]));

  if ( !ct_self_test (&ct, &self_test_readings) )
  {
    uart_logger_log_line ("CT self test failed.");
    tx_thread_suspend (this_thread);
  }

  uart_logger_log_line ("CT initialization complete. Temp = %3f, Salinity = %3f",
                        self_test_readings.temp, self_test_readings.salinity);
  (void) tx_event_flags_set (&initialization_flags, CT_INIT_SUCCESS, TX_OR);

  tx_thread_suspend (this_thread);

  watchdog_register_thread (CT_THREAD);
  watchdog_check_in (CT_THREAD);

  //
  // Run tests if needed
  if ( tests.ct_thread_test != NULL )
  {
    tests.ct_thread_test (NULL);
  }

  watchdog_check_in ();

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

    watchdog_check_in ();

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

    watchdog_check_in ();
    jump_to_waves ();
  }

// Take our samples
  ct_parsing_error_counter = 0;
  while ( ct->total_samples < configuration.total_ct_samples )
  {

    watchdog_check_in ();
    ct_return_code = ct->parse_sample ();

    if ( ct_return_code == CT_PARSING_ERROR )
    {
      ct_parsing_error_counter++;
    }

    if ( (ct_parsing_error_counter >= 10) || (ct_return_code == CT_UART_ERROR) )
    {
      // If there are too many parsing errors or a UART error occurs, then
      // stop trying and
      watchdog_check_in ();
      jump_to_waves ();
    }

    if ( ct_return_code == CT_DONE_SAMPLING )
    {
      break;
    }
  }

  watchdog_check_in ();

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
    watchdog_check_in ();
    jump_to_waves ();
  }

// Now set the mean salinity and temp values to the real ones
  half_salinity = floatToHalf ((float) ct->averages.salinity);
  half_temp = floatToHalf ((float) ct->averages.temp);

  memcpy (&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
  memcpy (&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

  watchdog_check_in (CT_THREAD);
  watchdog_deregister_thread (CT_THREAD);

  (void) tx_event_flags_set ()

  tx_thread_terminate (this_thread);
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
                    &error_flags,
                    TEMP_FET_GPIO_Port,
                    TEMP_FET_Pin, true);

  if ( !temperature_self_test (&temperature) )
  {
    uart_logger_log_line ("Temperature self test failed.");
    tx_thread_suspend (this_thread);
  }

  // TODO: add temperature and salinity readings to self test, report here.
  uart_logger_log_line ("Temperature initialization complete.");
  (void) tx_event_flags_set (&initialization_flags, TEMPERATURE_INIT_SUCCESS, TX_OR);

  tx_thread_suspend (this_thread);

  watchdog_register_thread (TEMPERATURE_THREAD);
  watchdog_check_in (TEMPERATURE_THREAD);

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  watchdog_check_in ();

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

  watchdog_check_in (TEMPERATURE_THREAD);
  watchdog_deregister_thread (TEMPERATURE_THREAD);

  tx_thread_terminate (this_thread);
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

  // TODO: init and self test

  tx_thread_suspend (this_thread);

  watchdog_register_thread (LIGHT_THREAD);
  watchdog_check_in (LIGHT_THREAD);

  // TODO: Run sensor

  watchdog_check_in (LIGHT_THREAD);
  watchdog_deregister_thread (LIGHT_THREAD);

  tx_thread_terminate (this_thread);
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

  tx_thread_suspend (this_thread);

  // TODO: init and self test

  tx_thread_suspend (this_thread);

  watchdog_register_thread (TURBIDITY_THREAD);
  watchdog_check_in (TURBIDITY_THREAD);

  // TODO: Run sensor

  watchdog_check_in (TURBIDITY_THREAD);
  watchdog_deregister_thread (TURBIDITY_THREAD);

  tx_thread_terminate (this_thread);
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

  // TODO: init and self test

  tx_thread_suspend (this_thread);

  watchdog_register_thread (ACCELEROMETER_THREAD);
  watchdog_check_in (ACCELEROMETER_THREAD);

  // TODO: Run sensor

  watchdog_check_in (ACCELEROMETER_THREAD);
  watchdog_deregister_thread (ACCELEROMETER_THREAD);

  tx_thread_terminate (&this_thread);
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

  watchdog_register_thread (WAVES_THREAD);
  watchdog_check_in (WAVES_THREAD);

  //
  // Run tests if needed
  if ( tests.temperature_thread_test != NULL )
  {
    tests.temperature_thread_test (NULL);
  }

  watchdog_check_in ();

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
  NEDwaves_memlight (north, east, down, gnss.sample_window_freq, &Hs, &Tp, &Dp, E, &b_fmin, &b_fmax,
                     a1, b1, a2, b2, check);

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

  watchdog_check_in (WAVES_THREAD);
  watchdog_deregister_thread (WAVES_THREAD);

  tx_thread_terminate (this_thread);
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

  if ( !iridium_apply_config (&iridium) )
  {
    uart_logger_log_line ("Iridium modem failed to initialize.");
    tx_thread_suspend (this_thread);
  }

  uart_logger_log_line ("Iridium modem initialized successfully.");
  (void) tx_event_flags_set (&initialization_flags, IRIDIUM_INIT_SUCCESS, TX_OR);

  tx_thread_suspend (this_thread);

  watchdog_register_thread (IRIDIUM_THREAD);
  watchdog_check_in (IRIDIUM_THREAD);

  UINT tx_return;
  int fail_counter;
  char ascii_7 = '7';
  uint8_t sbd_type = 52;
  uint16_t sbd_size = 327;
  real16_T voltage;
  float sbd_timestamp = iridium->get_timestamp ();
  bool queue_empty = iridium->storage_queue->num_msgs_enqueued == 0;

  watchdog_check_in ();

//
// Run tests if needed
  if ( tests.iridium_thread_test != NULL )
  {
    tests.iridium_thread_test (NULL);
  }

  watchdog_check_in ();

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

    watchdog_check_in ();
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

  watchdog_check_in ();

  iridium->transmit_message ();

  watchdog_check_in ();

// Check for error messages
  tx_event_flags_get (&error_flags, error_occured_flags, TX_OR_CLEAR, &actual_error_flags,
  TX_NO_WAIT);

// If we have an error flag, send an error message
  if ( actual_error_flags )
  {
    watchdog_check_in ();
    send_error_message (actual_error_flags);

    watchdog_check_in ();
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

  watchdog_check_in (IRIDIUM_THREAD);
  watchdog_deregister_thread (IRIDIUM_THREAD);

  tx_thread_terminate (this_thread);
}
/* USER CODE END 1 */

/*
 * watchdog.c
 *
 *  Created on: Jun 14, 2024
 *      Author: philbush
 */

#include "watchdog.h"
#include "configuration.h"
#include "threadx_support.h"

static watchdog_handle hidden_handle;

int32_t watchdog_init ( watchdog_handle wd_handle, TX_EVENT_FLAGS_GROUP *watchdog_check_in_flags )
{
  int32_t ret = WATCHDOG_OK;
  // Needed for the callback function
  hidden_handle = wd_handle;

  wd_handle->check_in_flags = watchdog_check_in_flags;
  wd_handle->adc_shutdown = false;
  wd_handle->acc_mag_shutdown = false;
  wd_handle->acc_gyro_shutdown = false;
  wd_handle->filex_shutdown = false;
  wd_handle->apf11_comms_shutdown = false;

  return ret;
}

/**
 * @brief  Callback which checks the event flags set and refreshes the watchdog accordingly
 * @param  flags_group_ptr := pointer to the watchdog flags group
 * @retval None
 */
void watchdog_event_callback ( TX_EVENT_FLAGS_GROUP *flags_group_ptr )
{
  UINT retval;
  ULONG current_flags, required_flags = 0;

  if ( !hidden_handle->adc_shutdown )
  {
    required_flags |= ADC_CHECK_IN;
  }

  if ( !hidden_handle->acc_mag_shutdown )
  {
    required_flags |= ACC_MAG_CHECK_IN;
  }

  if ( !hidden_handle->acc_gyro_shutdown )
  {
    required_flags |= ACC_GYRO_CHECK_IN;
  }

  if ( !hidden_handle->filex_shutdown )
  {
    required_flags |= FILEX_CHECK_IN;
  }

  if ( !hidden_handle->apf11_comms_shutdown )
  {
    required_flags |= APF11_COMMS_CHECK_IN;
  }

  retval = tx_event_flags_get (flags_group_ptr, required_flags, TX_OR, &current_flags, TX_NO_WAIT);

  // All of the threads have not checked in
  if ( retval != TX_SUCCESS )
  {
    return;
  }

  // All threads have checked in. Refresh the watchdog and clear the flags
  HAL_IWDG_Refresh (&hiwdg);

  // Clear the event flags
  (void) clear_event_flags (flags_group_ptr);
}

/**
 * @brief  Register an event flag to indicate the thread has checked in
 * @param  thread := which thread is checking in
 * @retval Void
 */
void watchdog_check_in ( watchdog_handle wd_handle, enum watchdog_thread_flags thread )
{
  (void) tx_event_flags_set (wd_handle->check_in_flags, thread, TX_OR);
}

/**
 * @brief  Register a thread error so the watchdog can be refreshed without this thread checking in.
 * @param  thread := which thread is reporting an error
 * @retval Void
 */
void watchdog_register_error ( watchdog_handle wd_handle, enum thread which_thread )
{
  switch ( which_thread )
  {
    case ADC_THREAD:
      wd_handle->adc_shutdown = true;
      break;

    case ACC_MAG_THREAD:
      wd_handle->acc_mag_shutdown = true;
      break;

    case ACC_GYRO_THREAD:
      wd_handle->acc_gyro_shutdown = true;
      break;

    case APF11_COMMS_THREAD:
      wd_handle->apf11_comms_shutdown = true;
      break;

    case FILEX_THREAD:
      wd_handle->filex_shutdown = true;
      break;

    default:

  }
}

/**
 * @brief  Deregister a thread error so the watchdog will continue checking it before refreshing
 * @param  thread := which thread has cleared the error
 * @retval Void
 */
void watchdog_deregister_error ( watchdog_handle wd_handle, enum thread which_thread )
{
  switch ( which_thread )
  {
    case ADC_THREAD:
      wd_handle->adc_shutdown = false;
      break;

    case ACC_MAG_THREAD:
      wd_handle->acc_mag_shutdown = false;
      break;

    case ACC_GYRO_THREAD:
      wd_handle->acc_gyro_shutdown = false;
      break;

    case APF11_COMMS_THREAD:
      wd_handle->apf11_comms_shutdown = false;
      break;

    case FILEX_THREAD:
      wd_handle->filex_shutdown = false;
      break;

    default: // ???
  }
}

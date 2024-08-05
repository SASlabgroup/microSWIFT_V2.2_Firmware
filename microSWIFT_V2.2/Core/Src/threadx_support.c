/*
 * threadx_support.c
 *
 *  Created on: Aug 5, 2024
 *      Author: philbush
 */

#include "threadx_support.h"
#include "app_threadx.h"
#include <math.h>

uint32_t ticks_from_milliseconds ( uint32_t milliseconds )
{
  if ( milliseconds == 0 )
  {
    return 0;
  }

  return ((uint32_t) (ceil (((float) milliseconds) / ((1.0f / TX_TIMER_TICKS_PER_SECOND) * 1000.0f))));
}

void clear_all_flags ( TX_EVENT_FLAGS_GROUP *flags_group )
{
  ULONG actual_flags;
  (void) tx_event_flags_get (flags_group, ALL_FLAGS, TX_OR_CLEAR, &actual_flags, TX_NO_WAIT);
}

ULONG get_current_flags ( TX_EVENT_FLAGS_GROUP *flags_group )
{
  ULONG actual_flags = 0;
  (void) tx_event_flags_get (flags_group, ALL_FLAGS, TX_OR, &actual_flags, TX_NO_WAIT);
  return actual_flags;
}

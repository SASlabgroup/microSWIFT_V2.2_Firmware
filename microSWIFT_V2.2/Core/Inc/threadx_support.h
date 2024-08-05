/*
 * threadx_support.h
 *
 *  Created on: Aug 5, 2024
 *      Author: philbush
 */

#ifndef INC_THREADX_SUPPORT_H_
#define INC_THREADX_SUPPORT_H_

#include "app_threadx.h"

#define ALL_FLAGS 0xFFFFFFFF

uint32_t ticks_from_milliseconds ( uint32_t milliseconds );
void clear_all_flags ( TX_EVENT_FLAGS_GROUP *flags_group );
uint32_t get_current_flags ( TX_EVENT_FLAGS_GROUP *flags_group );

#endif /* INC_THREADX_SUPPORT_H_ */

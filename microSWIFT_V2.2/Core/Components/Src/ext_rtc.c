/*
 * ext_rtc.c
 *
 *  Created on: Jul 29, 2024
 *      Author: philbush
 */

#include "ext_rtc.h"
#include "pcf2131_reg.h"

static int32_t ext_rtc_get_access_lock ( void );
static int32_t ext_rtc_release_access_lock ( void );

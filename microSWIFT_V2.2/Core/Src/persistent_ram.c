/*
 * persistent_ram.c
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#include "persistent_ram.h"

static Persistent_Storage self __attribute__((section(".sram2")));

void persistent_storage_init ( void )
{

}

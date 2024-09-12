/*
 * persistent_ram.h
 *
 *  Created on: Sep 12, 2024
 *      Author: philbush
 */

#ifndef INC_PERSISTENT_RAM_H_
#define INC_PERSISTENT_RAM_H_

#define PERSISTENT_RAM_MAGIC_DOUBLE_WORD 0x5048494C42555348

typedef struct
{
  uint64_t magic_word;
} Persistent_Storage;

void persistent_storage_init ( void );
void persistent_storage_clear ( void );

#endif /* INC_PERSISTENT_RAM_H_ */

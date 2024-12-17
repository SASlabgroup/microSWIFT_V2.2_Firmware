/*
 * shared_i2c_bus.h
 *
 *  Created on: Dec 13, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_INC_SHARED_I2C_BUS_H_
#define COMPONENTS_INC_SHARED_I2C_BUS_H_

#include "i2c.h"
#include "tx_api.h"
#include "microSWIFT_return_codes.h"

//@formatter:off
#define CORE_I2C_BUS            I2C2
#define I2C_QUEUE_LENGTH        25
#define I2C_OP_WAIT_TICKS       5

typedef enum
{
  I2C_READ  = 0,
  I2C_WRITE = 1
} i2c_operation_type_t;

typedef struct
{
  uint8_t                   *input_output_buffer;
  uSWIFT_return_code_t      *return_code;
  UINT                      complete_flag;
  uint16_t                  dev_addr;
  uint16_t                  dev_reg;
  uint16_t                  data_len;
  uint16_t                  operation_type; // type i2c_operation_type_t
} i2c_queue_message;

typedef struct
{
  I2C_HandleTypeDef     *i2c_handle;
  TX_SEMAPHORE          *i2c_sema;

  uSWIFT_return_code_t  (*read) ( uint8_t dev_addr, uint8_t dev_reg,
                                  uint8_t *input_output_buffer, uint8_t read_len,
                                  ULONG wait_ticks );
  uSWIFT_return_code_t  (*write) ( uint8_t dev_addr, uint8_t dev_reg,
                                   uint8_t *input_output_buffer, uint8_t write_len,
                                   ULONG wait_ticks );
} Shared_I2C_Bus;

typedef struct
{
  TX_QUEUE              *i2c_queue;
  TX_EVENT_FLAGS_GROUP  *operation_complete_flags;
} Shared_I2C_Server;

bool                    shared_i2c_init ( Shared_I2C_Bus *shared_bus, I2C_HandleTypeDef *i2c_handle, TX_SEMAPHORE *i2c_int_sema );
void                    shared_i2c_server_init (TX_QUEUE *i2c_queue, TX_EVENT_FLAGS_GROUP *operation_complete_flags);
bool                    shared_i2c_deinit ( void );
uSWIFT_return_code_t    shared_i2c_read ( uint8_t dev_addr, uint8_t dev_reg,
                                          uint8_t *input_output_buffer, uint8_t read_len, UINT complete_flag );
uSWIFT_return_code_t    shared_i2c_write ( uint8_t dev_addr, uint8_t dev_reg,
                                           uint8_t *input_output_buffer, uint8_t write_len, UINT complete_flag );

//@formatter:on
#endif /* COMPONENTS_INC_SHARED_I2C_BUS_H_ */

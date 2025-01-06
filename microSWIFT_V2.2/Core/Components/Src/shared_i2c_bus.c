/*
 * shared_i2c_bus->c
 *
 *  Created on: Dec 13, 2024
 *      Author: philbush
 */

#include "shared_i2c_bus.h"

static Shared_I2C_Bus *i2c_bus;
static Shared_I2C_Server i2c_server;

static uSWIFT_return_code_t __shared_i2c_read ( uint8_t dev_addr, uint8_t dev_reg,
                                                uint8_t *input_output_buffer, uint8_t read_len,
                                                ULONG wait_ticks );
static uSWIFT_return_code_t __shared_i2c_write ( uint8_t dev_addr, uint8_t dev_reg,
                                                 uint8_t *input_output_buffer, uint8_t write_len,
                                                 ULONG wait_ticks );

bool shared_i2c_init ( Shared_I2C_Bus *shared_bus, I2C_HandleTypeDef *i2c_handle,
                       TX_SEMAPHORE *i2c_int_sema )
{
  i2c_bus = shared_bus;

  i2c_bus->i2c_handle = i2c_handle;
  i2c_bus->i2c_sema = i2c_int_sema;

  i2c_bus->read = __shared_i2c_read;
  i2c_bus->write = __shared_i2c_write;

  return (i2c2_init () == I2C_OK);
}

void shared_i2c_server_init ( TX_QUEUE *i2c_queue, TX_EVENT_FLAGS_GROUP *operation_complete_flags )
{
  i2c_server.i2c_queue = i2c_queue;
  i2c_server.operation_complete_flags = operation_complete_flags;
}

bool shared_i2c_deinit ( void )
{
  return (i2c_deinit () == I2C_OK);
}

uSWIFT_return_code_t shared_i2c_read ( uint8_t dev_addr, uint8_t dev_reg,
                                       uint8_t *input_output_buffer, uint8_t read_len,
                                       UINT complete_flag )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  i2c_queue_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.input_output_buffer = input_output_buffer;
  queue_msg.return_code = &ret;
  queue_msg.complete_flag = complete_flag;
  queue_msg.dev_addr = dev_addr;
  queue_msg.dev_reg = dev_reg;
  queue_msg.data_len = read_len;
  queue_msg.operation_type = I2C_READ;

  if ( tx_queue_send (i2c_server.i2c_queue, &queue_msg, 1) != TX_SUCCESS )
  {
    return uSWIFT_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (i2c_server.operation_complete_flags, complete_flag, TX_OR_CLEAR,
                           &event_flags, I2C_OP_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_TIMEOUT;
  }

  return ret;
}

uSWIFT_return_code_t shared_i2c_write ( uint8_t dev_addr, uint8_t dev_reg,
                                        uint8_t *input_output_buffer, uint8_t write_len,
                                        UINT complete_flag )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;
  i2c_queue_message queue_msg =
    { 0 };
  ULONG event_flags;

  queue_msg.input_output_buffer = input_output_buffer;
  queue_msg.return_code = &ret;
  queue_msg.complete_flag = complete_flag;
  queue_msg.dev_addr = dev_addr;
  queue_msg.dev_reg = dev_reg;
  queue_msg.data_len = write_len;
  queue_msg.operation_type = I2C_WRITE;

  if ( tx_queue_send (i2c_server.i2c_queue, &queue_msg, 1) != TX_SUCCESS )
  {
    return uSWIFT_MESSAGE_QUEUE_ERROR;
  }

  if ( tx_event_flags_get (i2c_server.operation_complete_flags, complete_flag, TX_OR_CLEAR,
                           &event_flags, I2C_OP_WAIT_TICKS)
       != TX_SUCCESS )
  {
    return uSWIFT_TIMEOUT;
  }

  return ret;
}

static uSWIFT_return_code_t __shared_i2c_read ( uint8_t dev_addr, uint8_t dev_reg,
                                                uint8_t *input_output_buffer, uint8_t read_len,
                                                ULONG wait_ticks )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( HAL_I2C_Mem_Read_IT (i2c_bus->i2c_handle, dev_addr, dev_reg, 1, input_output_buffer,
                            read_len)
       != HAL_OK )
  {
    ret = uSWIFT_IO_ERROR;
    return ret;
  }

  if ( tx_semaphore_get (i2c_bus->i2c_sema, wait_ticks) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMEOUT;
  }

  return ret;
}

static uSWIFT_return_code_t __shared_i2c_write ( uint8_t dev_addr, uint8_t dev_reg,
                                                 uint8_t *input_output_buffer, uint8_t write_len,
                                                 ULONG wait_ticks )
{
  uSWIFT_return_code_t ret = uSWIFT_SUCCESS;

  if ( write_len == 0 )
  {
    if ( HAL_I2C_Master_Transmit_IT (i2c_bus->i2c_handle, dev_addr, input_output_buffer, 1)
         != HAL_OK )
    {
      ret = uSWIFT_IO_ERROR;
    }
  }
  else
  {
    if ( HAL_I2C_Mem_Write_IT (i2c_bus->i2c_handle, dev_addr, dev_reg, 1, input_output_buffer,
                               write_len)
         != HAL_OK )
    {
      ret = uSWIFT_IO_ERROR;
    }
  }

  if ( tx_semaphore_get (i2c_bus->i2c_sema, wait_ticks) != TX_SUCCESS )
  {
    ret = uSWIFT_TIMEOUT;
  }

  return ret;
}

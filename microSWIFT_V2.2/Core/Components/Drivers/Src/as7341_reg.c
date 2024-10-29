/*
 * as7341_reg.c
 *
 *  Created on: Oct 24, 2024
 *      Author: philbush
 */

#include "as7341_reg.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
  struct smux_addr_0x00
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f3_left :3;
    uint8_t unused1 :1;
  } addr_0x00;

  struct smux_addr_0x01
  {
    as7341_smux_assignment_t f1_left :3;
    uint8_t unused :5;
  } addr_0x01;

  struct smux_addr_0x02
  {
    uint8_t unused :8;
  } addr_0x02;

  struct smux_addr_0x03
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f8_left :3;
    uint8_t unused1 :1;
  } addr_0x03;

  struct smux_addr_0x04
  {
    as7341_smux_assignment_t f6_left :3;
    uint8_t unused :5;
  } addr_0x04;

  struct smux_addr_0x05
  {
    as7341_smux_assignment_t f2_left :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t f4_left :3;
    uint8_t unuse1 :1;
  } addr_0x05;

  struct smux_addr_0x06
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f5_left :3;
    uint8_t unused1 :1;
  } addr_0x06;

  struct smux_addr_0x07
  {
    as7341_smux_assignment_t f7_left :3;
    uint8_t unused :5;
  } addr_0x07;

  struct smux_addr_0x08
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t clear_left :3;
    uint8_t unused1 :1;
  } addr_0x08;

  struct smux_addr_0x09
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f5_right :3;
    uint8_t unused1 :1;
  } addr_0x09;

  struct smux_addr_0x0A
  {
    as7341_smux_assignment_t f7_right :3;
    uint8_t unused :5;
  } addr_0x0a;

  struct smux_addr_0x0B
  {
    uint8_t unused :8;
  } addr_0x0b;

  struct smux_addr_0x0C
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f2_right :3;
    uint8_t unused1 :1;
  } addr_0x0c;

  struct smux_addr_0x0D
  {
    as7341_smux_assignment_t f4_right :3;
    uint8_t unused :5;
  } addr_0x0d;

  struct smux_addr_0x0E
  {
    as7341_smux_assignment_t f8_left :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t f6_left :3;
    uint8_t unuse1 :1;
  } addr_0x0e;

  struct smux_addr_0x0F
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t f3_right :3;
    uint8_t unused1 :1;
  } addr_0x0f;

  struct smux_addr_0x10
  {
    as7341_smux_assignment_t f1_right :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t ext_gpio :3;
    uint8_t unuse1 :1;
  } addr_0x10;

  struct smux_addr_0x11
  {
    as7341_smux_assignment_t ext_int :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t clear_right :3;
    uint8_t unuse1 :1;
  } addr_0x11;

  struct smux_addr_0x12
  {
    uint8_t unused0 :4;
    as7341_smux_assignment_t dark :3;
    uint8_t unused1 :1;
  } addr_0x12;

  struct smux_addr_0x13
  {
    as7341_smux_assignment_t nir :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t flicker :3;
    uint8_t unuse1 :1;
  } addr_0x13;

} as7341_smux_memory;

int32_t as7341_register_io_functions ( dev_ctx_t *dev_handle, dev_init_ptr init_fn,
                                       dev_deinit_ptr deinit_fn, dev_write_ptr bus_write_fn,
                                       dev_read_ptr bus_read_fn, dev_ms_delay_ptr delay,
                                       void *optional_handle )
{
  dev_handle->init = init_fn;
  dev_handle->deinit = deinit_fn;
  dev_handle->bus_read = bus_read_fn;
  dev_handle->bus_write = bus_write_fn;
  dev_handle->handle = optional_handle;
  dev_handle->delay = delay;

  return dev_handle->init ();
}

int32_t as7341_get_id ( dev_ctx_t *dev_handle, uint8_t *id )
{
  int32_t ret = AS7341_OK;

  ret = dev_handle->bus_read (NULL, AS7341_I2C_ADDR, ID_REG_ADDR, id, 1);

  return ret;
}

int32_t as7341_set_integration_mode ( dev_ctx_t *dev_handle, as7341_int_mode_t mode )
{
  int32_t ret = AS7341_OK;
  as7341_config_reg_t config_reg;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CONFIG_REG_ADDR, (uint8_t*) &config_reg, 1);

  config_reg.int_mode = mode;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, CONFIG_REG_ADDR, (uint8_t*) &config_reg, 1);

  return ret;
}

int32_t as7341_config_smux ( dev_ctx_t *dev_handle, as7341_smux_assignment *smux_assignment )
{
  int32_t ret = AS7341_OK;

  return ret;
}

int32_t as7341_power ( dev_ctx_t *dev_handle, bool on )
{
  int32_t ret = AS7341_OK;
  as7341_enable_reg_t enable_reg;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  enable_reg.pon = on;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  return ret;
}

int32_t as7341_smux_enable ( dev_ctx_t *dev_handle )
{
  int32_t ret = AS7341_OK;
  as7341_enable_reg_t enable_reg;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  enable_reg.smuxen = PROPERTY_ENABLE;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  return ret;
}

int32_t as7341_wait_config ( dev_ctx_t *dev_handle, bool enable )
{
  int32_t ret = AS7341_OK;
  as7341_enable_reg_t enable_reg;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  enable_reg.wen = enable;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  return ret;
}

int32_t as7341_spectral_meas_config ( dev_ctx_t *dev_handle, bool enable )
{
  int32_t ret = AS7341_OK;
  as7341_enable_reg_t enable_reg;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  enable_reg.sp_en = enable;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ENABLE_REG_ADDR, (uint8_t*) &enable_reg, 1);

  return ret;
}

int32_t as7341_set_wait_time ( dev_ctx_t *dev_handle, float wait_time_ms )
{
  int32_t ret = AS7341_OK;
  as7341_wtime_reg_t wait_time;

  if ( wait_time_ms < WTIME_MIN_VAL )
  {
    wait_time.wtime = 0;
  }
  else if ( wait_time_ms > WTIME_MAX_VAL )
  {
    wait_time.wtime = 255;
  }
  else
  {
    wait_time.wtime = WTIME_FROM_MS(wait_time_ms);
  }

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, WTIME_REG_ADDR, (uint8_t*) &wait_time, 1);

  return ret;
}

int32_t as7341_set_atime ( dev_ctx_t *dev_handle, uint8_t atime )
{
  int32_t ret = AS7341_OK;
  as7341_atime_reg_t time;

  time.atime = atime;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ATIME_REG_ADDR, (uint8_t*) &time, 1);

  return ret;
}

int32_t as7341_set_astep ( dev_ctx_t *dev_handle, uint16_t astep )
{
  int32_t ret = AS7341_OK;
  as7341_astep_t step;

  step.astep = astep;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ASTEP_LOWER_REG_ADDR,
                                (uint8_t*) &step.astep_struct.astep_lower, 1);
  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, ASTEP_UPPER_REG_ADDR,
                                (uint8_t*) &step.astep_struct.astep_upper, 1);

  return ret;
}

int32_t as7341_set_again ( dev_ctx_t *dev_handle, as7341_again_t gain_setting )
{
  int32_t ret = AS7341_OK;
  as7341_cfg1_reg_t cfg1;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CFG1_REG_ADDR, (uint8_t*) &cfg1, 1);

  cfg1.again = gain_setting;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, CFG1_REG_ADDR, (uint8_t*) &cfg1, 1);

  return ret;
}

int32_t as7341_get_all_channel_data ( dev_ctx_t *dev_handle,
                                      as7341_all_channel_data_struct *channel_data )
{
  int32_t ret = AS7341_OK;

  return ret;
}

int32_t as7341_set_register_bank ( dev_ctx_t *dev_handle, as7341_reg_bank_t bank )
{
  int32_t ret = AS7341_OK;
  as7341_cfg0_reg_t cfg0;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CFG0_REG_ADDR, (uint8_t*) &cfg0, 1);

  cfg0.reg_bank = bank;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, CFG0_REG_ADDR, (uint8_t*) &cfg0, 1);

  return ret;
}


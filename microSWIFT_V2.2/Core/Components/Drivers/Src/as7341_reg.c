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
    as7341_smux_assignment_t f8_right :3;
    uint8_t unuse0 :1;
    as7341_smux_assignment_t f6_right :3;
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
                                       as7341_gpio_handle gpio_handle )
{
  dev_handle->init = init_fn;
  dev_handle->deinit = deinit_fn;
  dev_handle->bus_read = bus_read_fn;
  dev_handle->bus_write = bus_write_fn;
  dev_handle->handle = (void*) gpio_handle;
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
  as7341_smux_memory smux_memory =
    { SMUX_ASSIGNMENT_DISABLE };
  as7341_smux_channels_t adc_assignment;
  as7341_smux_assignment_t which_adc;

  for ( int i = 0; i < AS7341_NUM_ADCS; i++ )
  {
    adc_assignment = i + 1;
    adc_assignment = smux_assignment->adc_assignments[i];

    switch ( adc_assignment )
    {
      case ADC_DISABLE:
        break;

      case F1:
        smux_memory.addr_0x01.f1_left = adc_assignment;
        smux_memory.addr_0x10.f1_right = adc_assignment;
        break;

      case F2:
        smux_memory.addr_0x05.f2_left = adc_assignment;
        smux_memory.addr_0x0c.f2_right = adc_assignment;
        break;

      case F3:
        smux_memory.addr_0x00.f3_left = adc_assignment;
        smux_memory.addr_0x0f.f3_right = adc_assignment;
        break;

      case F4:
        smux_memory.addr_0x05.f4_left = adc_assignment;
        smux_memory.addr_0x0d.f4_right = adc_assignment;
        break;

      case F5:
        smux_memory.addr_0x06.f5_left = adc_assignment;
        smux_memory.addr_0x09.f5_right = adc_assignment;
        break;

      case F6:
        smux_memory.addr_0x04.f6_left = adc_assignment;
        smux_memory.addr_0x0e.f6_right = adc_assignment;
        break;

      case F7:
        smux_memory.addr_0x07.f7_left = adc_assignment;
        smux_memory.addr_0x0a.f7_right = adc_assignment;
        break;

      case F8:
        smux_memory.addr_0x03.f8_left = adc_assignment;
        smux_memory.addr_0x0e.f8_right = adc_assignment;
        break;

      case CLEAR:
        smux_memory.addr_0x08.clear_left = adc_assignment;
        smux_memory.addr_0x11.clear_right = adc_assignment;
        break;

      case NIR:
        smux_memory.addr_0x13.nir = adc_assignment;
        break;

      case FLICKER:
        smux_memory.addr_0x13.flicker = adc_assignment;
        break;

      case GPIO:
        smux_memory.addr_0x10.ext_gpio = adc_assignment;
        break;

      case EXT_INT:
        smux_memory.addr_0x11.ext_int = adc_assignment;
        break;

      case DARK:
        smux_memory.addr_0x12.dark = adc_assignment;
        break;

      default:
        return AS7341_ERROR;
    }
  }

  // Power on
  ret |= as7341_power (dev_handle, true);

  // Enable spectral interrupt so we can check when the smux command has completed
  ret |= as7341_config_smux_interrupt (dev_handle, true);

  // Enable system interrupts so the SMUX int will actually fire
  ret |= as7341_config_sys_interrupts (dev_handle, true);

  // Send SMUX command to write to SMUX memory
  ret |= as7341_send_smux_command (dev_handle, SMUX_WRITE_CONFIG_FROM_RAM);

  // Write to SMUX RAM
  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, SMUX_MEMORY_ADDR_LOW,
                                (uint8_t*) &smux_memory, SMUX_MEMORY_SIZE);

  // Enable the SMUX
  ret |= as7341_smux_enable (dev_handle);

  // Wait until the interrupt fires to let us know the SMUX has completed (INT pin is active low)
  while ( ((as7341_gpio_handle) dev_handle->handle)->get_int_pin_state != GPIO_PIN_RESET )
  {
    dev_handle->delay (1);
  }

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

int32_t as7341_send_smux_command ( dev_ctx_t *dev_handle, as7341_smux_cmd_t cmd )
{
  int32_t ret = AS7341_OK;
  as7341_cfg6_reg_t cfg6;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CFG6_REG_ADDR, (uint8_t*) &cfg6, 1);

  cfg6.smux_cmd = cmd;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, CFG6_REG_ADDR, (uint8_t*) &cfg6, 1);

  return ret;
}

int32_t as7341_config_smux_interrupt ( dev_ctx_t *dev_handle, bool enable )
{
  int32_t ret = AS7341_OK;
  as7341_cfg9_reg_t cfg9;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CFG9_REG_ADDR, (uint8_t*) &cfg9, 1);

  cfg9.sien_smux = enable;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, CFG9_REG_ADDR, (uint8_t*) &cfg9, 1);

  return ret;
}

int32_t as7341_config_sys_interrupts ( dev_ctx_t *dev_handle, bool enable )
{
  int32_t ret = AS7341_OK;
  as7341_intenab_reg_t inten;

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, INTENAB_REG_ADDR, (uint8_t*) &inten, 1);

  inten.sien = enable;

  ret |= dev_handle->bus_write (NULL, AS7341_I2C_ADDR, INTENAB_REG_ADDR, (uint8_t*) &inten, 1);

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

  ret |= dev_handle->bus_read (NULL, AS7341_I2C_ADDR, CH0_LOWER_REG_ADDR, (uint8_t*) &channel_data,
                               sizeof(as7341_all_channel_data_struct));

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


/*
 * as7341_reg.c
 *
 *  Created on: Oct 24, 2024
 *      Author: philbush
 */

#include "as7341_reg.h"

static typedef struct
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

static int32_t as7341_set_register_bank ( dev_ctx_t *dev_handle, as7341_reg_bank_t bank );

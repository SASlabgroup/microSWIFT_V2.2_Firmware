/*
 * as7341_reg.h
 *
 *  Created on: Oct 24, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_DRIVERS_INC_AS7341_REG_H_
#define COMPONENTS_DRIVERS_INC_AS7341_REG_H_

#include "stdint.h"
#include "stdbool.h"
#include "reg_driver_def.h"

/**************************************************************************************************/
/************************************** Misc Definitions ******************************************/
/**************************************************************************************************/
#define AS7341_I2C_ADDR 0b01110010 // 0x39 << 1

/**************************************************************************************************/
/**************************************** Return Codes ********************************************/
/**************************************************************************************************/
#define AS7341_OK 0
#define AS7341_ERROR -1

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Register Definitions ########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/**************************************************************************************************/
/*********************************** Control Register 1 *******************************************/
/**************************************************************************************************/
#define ASTATUS_REG_ADDR (0x60)
#define ASTATUS_REG_RESET_VAL (0b00000000)

//typedef struct
//{
//#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
//  uint8_t second_irq_en :1;
//  uint8_t minute_irq_en :1;
//  hours_mode_t hours_mode :1;
//  uint8_t por_override :1;
//  uint8_t one_100_sec_disable :1;
//  uint8_t stop :1;
//  uint8_t temp_comp_disable :1;
//  uint8_t ext_clock_test_en :1;
//#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
//  uint8_t ext_clock_test_enable :1;
//  uint8_t temp_comp_disable :1;
//  uint8_t stop :1;
//  uint8_t one_100_sec_disable :1;
//  uint8_t por_override :1;
//  hours_mode_t hours_mode :1;
//  uint8_t minute_irq :1;
//  uint8_t second_irq :1;
//#endif /* DRV_BYTE_ORDER */
//} as7341_astatus_reg_t;

#endif /* COMPONENTS_DRIVERS_INC_AS7341_REG_H_ */

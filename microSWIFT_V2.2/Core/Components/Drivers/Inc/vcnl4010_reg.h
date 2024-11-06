/*
 * vcnl4010_reg.h
 *
 *  Created on: Nov 5, 2024
 *      Author: philbush
 */

#ifndef COMPONENTS_DRIVERS_INC_VCNL4010_REG_H_
#define COMPONENTS_DRIVERS_INC_VCNL4010_REG_H_

//@formatter:off
/**************************************************************************************************/
/************************************** Misc Definitions ******************************************/
/**************************************************************************************************/
#define VCNL4010_I2C_ADDR 0b00100110 // 0x13 << 1

/**************************************************************************************************/
/**************************************** Return Codes ********************************************/
/**************************************************************************************************/
#define VCNL4010_OK 0
#define VCNL4010_ERROR -1

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Register Definitions ########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#warning"READ ALL REGISTERS TO FIGURE OUT DEFAULT VALUES"
/**************************************************************************************************/
/************************************ Command Register ********************************************/
/**************************************************************************************************/
#define COMMAND_0_REG_ADDR (0x80)
#define COMMAND_0_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   selftimed_en    :1;
  uint8_t   prox_en         :1;
  uint8_t   als_en          :1;
  uint8_t   prox_od         :1;
  uint8_t   als_od          :1;
  uint8_t   prox_data_rdy   :1;
  uint8_t   als_data_rdy    :1;
  uint8_t   config_lock     :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   config_lock     :1;
  uint8_t   als_data_rdy    :1;
  uint8_t   prox_data_rdy   :1;
  uint8_t   als_od          :1;
  uint8_t   prox_od         :1;
  uint8_t   als_en          :1;
  uint8_t   prox_en         :1;
  uint8_t   selftimed_en    :1;
#endif /* DRV_BYTE_ORDER */
} vcnl4010_command_0_reg_t;

/**************************************************************************************************/
/************************ Product ID Revision Register ********************************************/
/**************************************************************************************************/
#define PROD_ID_REV_REG_ADDR (0x81)
#define PROD_ID_REV_REG_RESET_VAL (0b00100001)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   revision_id     :4;
  uint8_t   product_id      :4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   product_id      :4;
  uint8_t   revision_id     :4;
#endif /* DRV_BYTE_ORDER */
} vcnl4010_prod_id_rev_reg_t;

/**************************************************************************************************/
/*************************** Proximity Rate Register **********************************************/
/**************************************************************************************************/
#define PROXIMITY_RATE_REG_ADDR (0x82)
#define PROXIMITY_RATE_REG_RESET_VAL (0b00000000)

typedef enum
{
  _1_95_MEAS_PER_SEC    = 0B000,
  _3_90625_MEAS_PER_SEC = 0B001,
  _7_8125_MEAS_PER_SEC  = 0B010,
  _16_625_MEAS_PER_SEC  = 0B011,
  _31_25_MEAS_PER_SEC   = 0B100,
} vcnl4010_proximity_rate_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  vcnl4010_proximity_rate_t proximity_rate  :3;
  uint8_t                   reserved        :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t                   reserved        :5;
  vcnl4010_proximity_rate_t proximity_rate  :3;
#endif /* DRV_BYTE_ORDER */
} vcnl4010_proximity_rate_reg_t;


/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Function Declarations #######################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

//@formatter:on
#endif /* COMPONENTS_DRIVERS_INC_VCNL4010_REG_H_ */

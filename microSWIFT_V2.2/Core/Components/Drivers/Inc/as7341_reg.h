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

// @formatter:off

/**************************************************************************************************/
/************************************** Misc Definitions ******************************************/
/**************************************************************************************************/
#define AS7341_I2C_ADDR 0b01110010 // 0x39 << 1

typedef enum
{
  GAIN_0_5X     = 0B0000,
  GAIN_1X       = 0B0001,
  GAIN_2X       = 0B0010,
  GAIN_4X       = 0B0011,
  GAIN_8X       = 0B0100,
  GAIN_16X      = 0B0101,
  GAIN_32X      = 0B0110,
  GAIN_64X      = 0B0111,
  GAIN_128X     = 0B1000,
  GAIN_256X     = 0B1001,
  GAIN_512X     = 0B1010
} as7341_again_t;

typedef struct
{
  uint8_t   ch_lower_byte;
  uint8_t   ch_upper_byte;
} as7341_channel_data_struct;

typedef union
{
  as7341_channel_data_struct    channel_struct;
  uint16_t                      raw_counts;
} as7341_channel_data_t;

/**************************************************************************************************/
/**************************************** Return Codes ********************************************/
/**************************************************************************************************/
#define AS7341_OK 0
#define AS7341_ERROR -1

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Register Definitions ########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/* Registers are listed in order of their address, not the order in the datasheet or logical order
 * as AMS has scattered functionality across registers in a non-sensical manner.
 */

/**************************************************************************************************/
/**************************************  Register *********************************************/
/**************************************************************************************************/
#define _REG_ADDR (0X)
#define _REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN

#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN

#endif /* DRV_BYTE_ORDER */
} as7341__reg_t;

/**************************************************************************************************/
/*********************************** Control Register 1 *******************************************/
/**************************************************************************************************/
#define ASTATUS_REG_ADDR (0x60)
#define ASTATUS_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_again_t    again_status :4;
  uint8_t           reserved     :3;
  uint8_t           astat_status :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t           astat_status :1;
  uint8_t           reserved     :3;
  as7341_again_t    again_status :4;
#endif /* DRV_BYTE_ORDER */
} as7341_astatus_reg_t;

/**************************************************************************************************/
/******************************** Channel 0 data Register *****************************************/
/**************************************************************************************************/
#define CH0_LOWER_REG_ADDR (0x61)
#define CH0_UPPER_REG_ADDR (0X62)
#define CH0_LOWER_REG_RESET_VAL (0b00000000)
#define CH0_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/************************************* ITime Register *********************************************/
/**************************************************************************************************/
#define ITIME_L_REG_ADDR (0X63)
#define ITIME_M_REG_ADDR (0X64)
#define ITIME_H_REG_ADDR (0X65)
#define ITIME_L_REG_RESET_VAL (0b00000000)
#define ITIME_M_REG_RESET_VAL (0b00000000)
#define ITIME_H_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/******************************** Channel 1 data Register *****************************************/
/**************************************************************************************************/
#define CH1_LOWER_REG_ADDR (0x66)
#define CH1_UPPER_REG_ADDR (0X67)
#define CH1_LOWER_REG_RESET_VAL (0b00000000)
#define CH1_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/******************************** Channel 2 data Register *****************************************/
/**************************************************************************************************/
#define CH2_LOWER_REG_ADDR (0x68)
#define CH2_UPPER_REG_ADDR (0X69)
#define CH2_LOWER_REG_RESET_VAL (0b00000000)
#define CH2_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/******************************** Channel 3 data Register *****************************************/
/**************************************************************************************************/
#define CH3_LOWER_REG_ADDR (0x6A)
#define CH3_UPPER_REG_ADDR (0X6B)
#define CH3_LOWER_REG_RESET_VAL (0b00000000)
#define CH3_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/******************************** Channel 4 data Register *****************************************/
/**************************************************************************************************/
#define CH4_LOWER_REG_ADDR (0x6C)
#define CH4_UPPER_REG_ADDR (0X6D)
#define CH4_LOWER_REG_RESET_VAL (0b00000000)
#define CH4_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/******************************** Channel 5 data Register *****************************************/
/**************************************************************************************************/
#define CH5_LOWER_REG_ADDR (0x6E)
#define CH5_UPPER_REG_ADDR (0X6F)
#define CH5_LOWER_REG_RESET_VAL (0b00000000)
#define CH5_UPPER_REG_RESET_VAL (0b00000000)

/**************************************************************************************************/
/************************************ CONFIG Register *********************************************/
/**************************************************************************************************/
#define CONFIG_REG_ADDR (OX70)
#define CONFIG_REG_RESET_VAL (0b00000000)

typedef enum
{
  SPM_MODE  = 0B00,
  SYNS_MODE = 0B01,
  SYND_MODE = 0B11
} as7341_int_mode_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_int_mode_t int_mode :2;
  uint8_t           int_sel  :1;
  uint8_t           led_sel  :1;
  uint8_t           reserved :4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t           reserved :4;
  uint8_t           led_sel  :1;
  uint8_t           int_sel  :1;
  as7341_int_mode_t int_mode :2;
#endif /* DRV_BYTE_ORDER */
} as7341_config_reg_t;

/**************************************************************************************************/
/************************************** STAT Register *********************************************/
/**************************************************************************************************/
#define STAT_REG_ADDR (0X71)
#define STAT_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   ready       :1;
  uint8_t   wait_sync   :1;
  uint8_t   reserved    :6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved    :6;
  uint8_t   wait_sync   :1;
  uint8_t   ready       :1;
#endif /* DRV_BYTE_ORDER */
} as7341_stat_reg_t;

/**************************************************************************************************/
/************************************** EDGE Register *********************************************/
/**************************************************************************************************/
#define EDGE_REG_ADDR (0X72)
#define EDGE_REG_RESET_VAL (0b00000000)

typedef struct
{
  uint8_t   sync_edge;
} as7341_edge_reg_t;

/**************************************************************************************************/
/************************************** GPIO Register *********************************************/
/**************************************************************************************************/
#define GPIO_REG_ADDR (0X73)
#define GPIO_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   pd_int  :1;
  uint8_t   pd_gpio :1;
  uint8_t   reserved:6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved:6;
  uint8_t   pd_gpio :1;
  uint8_t   pd_int  :1;
#endif /* DRV_BYTE_ORDER */
} as7341_gpio_reg_t;

/**************************************************************************************************/
/************************************** LED Register **********************************************/
/**************************************************************************************************/
#define LED_REG_ADDR (0X74)
#define LED_REG_RESET_VAL (0b00000100)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   led_drive   :7;
  uint8_t   led_act     :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   led_act     :1;
  uint8_t   led_drive   :7;
#endif /* DRV_BYTE_ORDER */
} as7341__reg_t;

/**************************************************************************************************/
/************************************* ENABLE Register ********************************************/
/**************************************************************************************************/
#define ENABLE_REG_ADDR (0X80)
#define ENABLE_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   pon         :1;
  uint8_t   sp_en       :1;
  uint8_t   reserved0   :1;
  uint8_t   wen         :1;
  uint8_t   smuxen      :1;
  uint8_t   reserved1   :1;
  uint8_t   fden        :1;
  uint8_t   reserved2   :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved2   :1;
  uint8_t   fden        :1;
  uint8_t   reserved1   :1;
  uint8_t   smuxen      :1;
  uint8_t   wen         :1;
  uint8_t   reserved0   :1;
  uint8_t   sp_en       :1;
  uint8_t   pon         :1;
#endif /* DRV_BYTE_ORDER */
} as7341_enable_reg_t;

/**************************************************************************************************/
/************************************** ATIME Register ********************************************/
/**************************************************************************************************/
#define ATIME_REG_ADDR (0X81)
#define ATIME_REG_RESET_VAL (0b00000000)

typedef struct
{
  uint8_t   atime;
} as7341_atime_reg_t;

/**************************************************************************************************/
/************************************** WTIME Register ********************************************/
/**************************************************************************************************/
#define WTIME_REG_ADDR (0X83)
#define WTIME_REG_RESET_VAL (0b00000000)

typedef struct
{
  uint8_t   wtime;
} as7341_wtime_reg_t;

/**************************************************************************************************/
/************************************ SP_TH_L/H Register ******************************************/
/**************************************************************************************************/
#define SP_TH_L_LSB_REG_ADDR (0X84)
#define SP_TH_L_MSB_REG_ADDR (0X85)
#define SP_TH_H_LSB_REG_ADDR (0X86)
#define SP_TH_H_MSB_REG_ADDR (0X87)
#define SP_TH_L_LSB_REG_RESET_VAL (0b00000000)
#define SP_TH_L_MSB_REG_RESET_VAL (0b00000000)
#define SP_TH_H_LSB_REG_RESET_VAL (0b00000000)
#define SP_TH_H_MSB_REG_RESET_VAL (0b00000000)

typedef struct
{
  uint8_t   sp_th_lsb;
  uint8_t   sp_th_msb;
} as7341_sp_th_struct;

typedef union
{
  as7341_sp_th_struct   spectral_threshold_struct;
  uint16_t              spectral_threshold;
} as7341_sp_th_t;

/**************************************************************************************************/
/************************************** AUXID Register ********************************************/
/**************************************************************************************************/
#define AUXID_REG_ADDR (0X90)
#define AUXID_REG_RESET_VAL (0b00000000)
#define AS7341_AUXID (0b000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   auxid       :3;
  uint8_t   reserved    :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved    :5;
  uint8_t   auxid       :3;
#endif /* DRV_BYTE_ORDER */
} as7341_auxid_reg_t;

/**************************************************************************************************/
/************************************** REVID Register ********************************************/
/**************************************************************************************************/
#define REVID_REG_ADDR (0X91)
#define REVID_REG_RESET_VAL (0b00000000)
#define AS7341_REVID (0b000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   rev_id      :3;
  uint8_t   reserved    :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved    :5;
  uint8_t   rev_id      :3;
#endif /* DRV_BYTE_ORDER */
} as7341_revid_reg_t;

/**************************************************************************************************/
/**************************************** ID Register *********************************************/
/**************************************************************************************************/
#define ID_REG_ADDR (0X92)
#define ID_REG_RESET_VAL (0b00001001)
#define AS7341_ID (0b001001)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved    :2;
  uint8_t   id          :6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   id          :6;
  uint8_t   reserved    :2;
#endif /* DRV_BYTE_ORDER */
} as7341_id_reg_t;

/**************************************************************************************************/
/************************************** STATUS Register *********************************************/
/**************************************************************************************************/
#define STATUS_REG_ADDR (0X93)
#define STATUS_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   sint        :1;
  uint8_t   c_int       :1;
  uint8_t   fint        :1;
  uint8_t   aint        :1;
  uint8_t   reserved    :3;
  uint8_t   asat        :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   asat        :1;
  uint8_t   reserved    :3;
  uint8_t   aint        :1;
  uint8_t   fint        :1;
  uint8_t   c_int       :1;
  uint8_t   sint        :1;
#endif /* DRV_BYTE_ORDER */
} as7341_status_reg_t;

/**************************************************************************************************/
/************************************ STATUS 2 Register *******************************************/
/**************************************************************************************************/
#define STATUS_2_REG_ADDR (0XA3)
#define STATUS_2_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   fdsat_digital       :1;
  uint8_t   fdsat_analog        :1;
  uint8_t   reserved0           :1;
  uint8_t   asat_analog         :1;
  uint8_t   ast_digital         :1;
  uint8_t   reserved1           :1;
  uint8_t   avalid              :1;
  uint8_t   reserved2           :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved2           :1;
  uint8_t   avalid              :1;
  uint8_t   reserved1           :1;
  uint8_t   ast_digital         :1;
  uint8_t   asat_analog         :1;
  uint8_t   reserved0           :1;
  uint8_t   fdsat_analog        :1;
  uint8_t   fdsat_digital       :1;
#endif /* DRV_BYTE_ORDER */
} as7341_status_2_reg_t;

/**************************************************************************************************/
/************************************ STATUS 3 Register *******************************************/
/**************************************************************************************************/
#define STATUS_3_REG_ADDR (0XA4)
#define STATUS_3_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0   :4;
  uint8_t   int_sp_l    :1;
  uint8_t   int_sp_h    :1;
  uint8_t   reserved1   :2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved1   :2;
  uint8_t   int_sp_h    :1;
  uint8_t   int_sp_l    :1;
  uint8_t   reserved0   :4;
#endif /* DRV_BYTE_ORDER */
} as7341_status_3_reg_t;

/**************************************************************************************************/
/************************************ STATUS 5 Register *******************************************/
/**************************************************************************************************/
#define STATUS_5_REG_ADDR (0XA6)
#define STATUS_5_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0   :2;
  uint8_t   sint_mux    :1;
  uint8_t   sint_fd     :1;
  uint8_t   reserved1   :4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved1   :4;
  uint8_t   sint_fd     :1;
  uint8_t   sint_mux    :1;
  uint8_t   reserved0   :2;
#endif /* DRV_BYTE_ORDER */
} as7341_status_5_reg_t;

/**************************************************************************************************/
/************************************ STATUS 6 Register *******************************************/
/**************************************************************************************************/
#define STATUS_6_REG_ADDR (0XA7)
#define STATUS_6_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   int_busy        :1;
  uint8_t   sai_active      :1;
  uint8_t   sp_trig         :1;
  uint8_t   reserved0       :1;
  uint8_t   fd_trig         :1;
  uint8_t   ovtemp          :1;
  uint8_t   reserved1       :1;
  uint8_t   fifo_ov         :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   fifo_ov         :1;
  uint8_t   reserved1       :1;
  uint8_t   ovtemp          :1;
  uint8_t   fd_trig         :1;
  uint8_t   reserved0       :1;
  uint8_t   sp_trig         :1;
  uint8_t   sai_active      :1;
  uint8_t   int_busy        :1;
#endif /* DRV_BYTE_ORDER */
} as7341_status_6_reg_t;


/**************************************************************************************************/
/*************************************  CFG0 Register *********************************************/
/**************************************************************************************************/
#define CFG0_REG_ADDR (0XA9)
#define CFG0_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0   :2;
  uint8_t   wlong       :1;
  uint8_t   reserved1   :1;
  uint8_t   reg_bank    :1;
  uint8_t   low_power   :1;
  uint8_t   reserved2   :2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved2   :2;
  uint8_t   low_power   :1;
  uint8_t   reg_bank    :1;
  uint8_t   reserved1   :1;
  uint8_t   wlong       :1;
  uint8_t   reserved0   :2;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg0_reg_t;

/**************************************************************************************************/
/*************************************  CFG1 Register *********************************************/
/**************************************************************************************************/
#define CFG1_REG_ADDR (0XAA)
#define CFG1_REG_RESET_VAL (0b00001001)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_again_t    again       :5;
  uint8_t           reserved    :3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t           reserved    :3;
  as7341_again_t    again       :5;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg1_reg_t;

/**************************************************************************************************/
/*************************************  CFG3 Register *********************************************/
/**************************************************************************************************/
#define CFG3_REG_ADDR (0XAC)
#define CFG3_REG_RESET_VAL (0b00001100)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0   :4;
  uint8_t   sai         :1;
  uint8_t   reserved1   :3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved1   :3;
  uint8_t   sai         :1;
  uint8_t   reserved0   :4;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg3_reg_t;

/**************************************************************************************************/
/*************************************  CFG6 Register *********************************************/
/**************************************************************************************************/
#define CFG6_REG_ADDR (0XAF)
#define CFG6_REG_RESET_VAL (0b00010000)

typedef enum
{
  SMUX_INIT_FROM_ROM            = 0b00,
  SMUX_READ_CONFIG_TO_RAM       = 0b01,
  SMUX_WRITE_CONFIG_FROM_RAM    = 0b10
} as7341_smux_cmd_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0           :3;
  uint8_t   as7341_smux_cmd_t   :2;
  uint8_t   reserved1           :3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved1           :3;
  uint8_t   as7341_smux_cmd_t   :2;
  uint8_t   reserved0           :3;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg6_reg_t;

/**************************************************************************************************/
/*************************************  CFG8 Register *********************************************/
/**************************************************************************************************/
#define CFG8_REG_ADDR (0XB1)
#define CFG8_REG_RESET_VAL (0b10001000)

typedef enum
{
  FIFO_LVL_1    = 0,
  FIFO_LVL_4    = 1,
  FIFO_LVL_8    = 2,
  FIFO_LVL_16   = 3
} as7341_fifo_threshold_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t                   reserved0   :2;
  uint8_t                   sp_agc      :1;
  uint8_t                   fd_agc      :1;
  uint8_t                   reserved1   :2;
  as7341_fifo_threshold_t   fifo_th     :2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN

#endif /* DRV_BYTE_ORDER */
} as7341_cfg8_reg_t;

// @formatter:on
#endif /* COMPONENTS_DRIVERS_INC_AS7341_REG_H_ */

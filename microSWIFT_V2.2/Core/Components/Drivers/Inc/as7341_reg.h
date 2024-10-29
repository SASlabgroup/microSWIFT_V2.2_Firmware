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

typedef enum
{
  REG_BANK_80_PLUS  = 0,
  REG_BANK_60_74    = 1
} as7341_reg_bank_t;

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
  as7341_fifo_threshold_t   fifo_th     :2;
  uint8_t                   reserved1   :2;
  uint8_t                   fd_agc      :1;
  uint8_t                   sp_agc      :1;
  uint8_t                   reserved0   :2;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg8_reg_t;

/**************************************************************************************************/
/*************************************  CFG9 Register *********************************************/
/**************************************************************************************************/
#define CFG9_REG_ADDR (0XB2)
#define CFG9_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved0   :4;
  uint8_t   sien_smux   :1;
  uint8_t   reserved1   :1;
  uint8_t   sien_fd     :1;
  uint8_t   reserved2   :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved2   :1;
  uint8_t   sien_fd     :1;
  uint8_t   reserved1   :1;
  uint8_t   sien_smux   :1;
  uint8_t   reserved0   :4;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg9_reg_t;

/**************************************************************************************************/
/*************************************  CFG10 Register ********************************************/
/**************************************************************************************************/
#define CFG10_REG_ADDR (0XB3)
#define CFG10_REG_RESET_VAL (0b11110010)

typedef enum
{
  PERCENT_50_H    = 0,
  PERCENT_62_5_H  = 1,
  PERCENT_75_H    = 2,
  PERCENT_82_5_H  = 3
} as7341_agc_high_threshold_t;

typedef enum
{
  PERCENT_12_5_L  = 0,
  PERCENT_25_L    = 1,
  PERCENT_37_5_L  = 2,
  PERCENT_50_L    = 3
} as7341_agc_low_threshold_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_agc_high_threshold_t   agc_h   :2;
  as7341_agc_low_threshold_t    agc_l   :2;
  uint8_t                       reserved:1;
  uint8_t                       fd_pers :3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t                       fd_pers :3;
  uint8_t                       reserved:1;
  as7341_agc_low_threshold_t    agc_l   :2;
  as7341_agc_high_threshold_t   agc_h   :2;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg10_reg_t;

/**************************************************************************************************/
/*************************************  CFG12 Register ********************************************/
/**************************************************************************************************/
#define CFG12_REG_ADDR (0XB5)
#define CFG12_REG_RESET_VAL (0b00000000)

typedef enum
{
  SPECTRAL_CH0  = 0,
  SPECTRAL_CH1  = 1,
  SPECTRAL_CH2  = 2,
  SPECTRAL_CH3  = 3,
  SPECTRAL_CH4  = 4
} as7341_spectral_channel_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_spectral_channel_t sp_th_ch    :3;
  uint8_t                   reserved    :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t                   reserved    :5;
  as7341_spectral_channel_t sp_th_ch    :3;
#endif /* DRV_BYTE_ORDER */
} as7341_cfg12_reg_t;

/**************************************************************************************************/
/**************************************  PERS Register ********************************************/
/**************************************************************************************************/
#define PERS_REG_ADDR (0XBD)
#define PERS_REG_RESET_VAL (0b00000000)

typedef enum
{
  EVERY_SPECTRAL_CYCLE  = 0,
  _1_OCCURANCE          = 1,
  _2_OCCURANCES         = 2,
  _3_OCCURANCES         = 3,
  _4_OCCURANCES         = 4,
  _5_OCCURANCES         = 5,
  _15_OCCURANCES        = 6,
  _20_OCCURANCES        = 7,
  _25_OCCURANCES        = 8,
  _30_OCCURNACES        = 9,
  _35_OCCURNACES        = 10,
  _40_OCCURANCES        = 11,
  _45_OCCURANCES        = 12,
  _50_OCCURNACES        = 13,
  _55_OCCURNACES        = 14,
  _60_OCCURNACES        = 15
} as7341_spectral_interrupt_persistence_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_spectral_interrupt_persistence_t   apers   :4;
  uint8_t                                   reserved:4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t                                   reserved:4;
  as7341_spectral_interrupt_persistence_t   apers   :4;
#endif /* DRV_BYTE_ORDER */
} as7341_pers_reg_t;

/**************************************************************************************************/
/************************************* GPIO_2 Register ********************************************/
/**************************************************************************************************/
#define GPIO_2_REG_ADDR (0XBE)
#define GPIO_2_REG_RESET_VAL (0b00000010)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   gpio_in     :1;
  uint8_t   gpio_out    :1;
  uint8_t   gpio_in_en  :1;
  uint8_t   gpio_inv    :1;
  uint8_t   reserved    :4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved    :4;
  uint8_t   gpio_inv    :1;
  uint8_t   gpio_in_en  :1;
  uint8_t   gpio_out    :1;
  uint8_t   gpio_in     :1;
#endif /* DRV_BYTE_ORDER */
} as7341_gpio_2_reg_t;

/**************************************************************************************************/
/************************************* ASTEP Register *********************************************/
/**************************************************************************************************/
#define ASTEP_LOWER_REG_ADDR (0XCA)
#define ASTEP_UPPER_REG_ADDR (0XCB)
#define ASTEP_LOWER_REG_RESET_VAL (0b11100111)
#define ASTEP_UPPER_REG_RESET_VAL (0b00000011)

typedef struct
{
  uint8_t   astep_lower;
  uint8_t   astep_upper;
} as7341_astep_struct;

typedef union
{
  as7341_astep_struct   astep_struct;
  uint16_t              astep;
} as7341_astep_t;

/**************************************************************************************************/
/********************************** AGC_GAIN_MAX Register *****************************************/
/**************************************************************************************************/
#define AGC_GAIN_MAX_REG_ADDR (0XCF)
#define AGC_GAIN_MAX_REG_RESET_VAL (0b01010101)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  as7341_again_t    agc_again_max   :4;
  as7341_again_t    agc_fd_gain_max :4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  as7341_again_t    agc_fd_gain_max :4;
  as7341_again_t    agc_again_max   :4;
#endif /* DRV_BYTE_ORDER */
} as7341_agc_gain_max_reg_t;

/**************************************************************************************************/
/************************************ AZ_CONFIG Register ******************************************/
/**************************************************************************************************/
#define AZ_CONFIG_REG_ADDR (0XD6)
#define AZ_CONFIG_REG_RESET_VAL (0b11111111)

typedef enum
{
  AUTO_ZERO_NEVER       = 0,
  AUTO_ZERO_ONLY_ONCE   = 255
} as7341_az_special_val_t;

typedef union
{
  as7341_az_special_val_t   special_val     :8;
  uint8_t                   nth_iteration   :8;
} as7341_az_iter_t;

typedef struct
{
  as7341_az_iter_t   az_nth_iteration;
} as7341_az_config_reg_t;

/**************************************************************************************************/
/************************************ FIFO_CFG0 Register ******************************************/
/**************************************************************************************************/
#define FIFO_CFG0_REG_ADDR (0XD7)
#define FIFO_CFG0_REG_RESET_VAL (0b00100001)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   reserved        : 7;
  uint8_t   fifo_write_fd   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   fifo_write_fd   : 1;
  uint8_t   reserved        : 7;
#endif /* DRV_BYTE_ORDER */
} as7341_fifo_cfg0_reg_t;


/**************************************************************************************************/
/************************************* FD_TIME Register *******************************************/
/**************************************************************************************************/
#define FD_TIME_1_REG_ADDR (0XD8)
#define FD_TIME_2_REG_ADDR (0XDA)
#define FD_TIME_1_REG_RESET_VAL (0b00000000)
#define FD_TIME_2_REG_RESET_VAL (0b01001000)

typedef struct
{
  uint8_t   fd_time_lsb     :8;
} as7341_fd_time_1_reg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t           fd_time_msb     :3;
  as7341_again_t    fd_gain         :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  as7341_again_t    fd_gain         :5;
  uint8_t           fd_time_msb     :3;
#endif /* DRV_BYTE_ORDER */
} as7341_fd_time_2_reg_t;

/**************************************************************************************************/
/*********************************** FD_STATUS Register *******************************************/
/**************************************************************************************************/
#define FD_STATUS_REG_ADDR (0XDB)
#define FD_STATUS_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   fd_100hz_flicker        :1;
  uint8_t   fd_120hz_flicker        :1;
  uint8_t   fd_100hz_flicker_valid  :1;
  uint8_t   fd_120hz_flicker_valid  :1;
  uint8_t   fd_saturation_detected  :1;
  uint8_t   fd_measurement_valid    :1;
  uint8_t   reserved                :2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved                :2;
  uint8_t   fd_measurement_valid    :1;
  uint8_t   fd_saturation_detected  :1;
  uint8_t   fd_120hz_flicker_valid  :1;
  uint8_t   fd_100hz_flicker_valid  :1;
  uint8_t   fd_120hz_flicker        :1;
  uint8_t   fd_100hz_flicker        :1;
#endif /* DRV_BYTE_ORDER */
} as7341_fd_status_reg_t;

/**************************************************************************************************/
/************************************** INTENAB Register *********************************************/
/**************************************************************************************************/
#define INTENAB_REG_ADDR (0XF9)
#define INTENAB_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   sien        :1;
  uint8_t   reserved0   :1;
  uint8_t   f_ien       :1;
  uint8_t   sp_ien      :1;
  uint8_t   reserved1   :3;
  uint8_t   asien       :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   asien       :1;
  uint8_t   reserved1   :3;
  uint8_t   sp_ien      :1;
  uint8_t   f_ien       :1;
  uint8_t   reserved0   :1;
  uint8_t   sien        :1;
#endif /* DRV_BYTE_ORDER */
} as7341_intenab_reg_t;

/**************************************************************************************************/
/************************************ CONTROL Register ********************************************/
/**************************************************************************************************/
#define CONTROL_REG_ADDR (0XFA)
#define CONTROL_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   clear_sai_act   :1;
  uint8_t   fifo_clr        :1;
  uint8_t   sp_man_az       :1;
  uint8_t   reserved        :5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved        :5;
  uint8_t   sp_man_az       :1;
  uint8_t   fifo_clr        :1;
  uint8_t   clear_sai_act   :1;
#endif /* DRV_BYTE_ORDER */
} as7341_control_reg_t;

/**************************************************************************************************/
/************************************ FIFO_MAP Register *******************************************/
/**************************************************************************************************/
#define FIFO_MAP_REG_ADDR (0XFC)
#define FIFO_MAP_REG_RESET_VAL (0b00000000)

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t   fifo_write_status   :1;
  uint8_t   fifo_write_ch0_data :1;
  uint8_t   fifo_write_ch1_data :1;
  uint8_t   fifo_write_ch2_data :1;
  uint8_t   fifo_write_ch3_data :1;
  uint8_t   fifo_write_ch4_data :1;
  uint8_t   fifo_write_ch5_data :1;
  uint8_t   reserved            :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t   reserved            :1;
  uint8_t   fifo_write_ch5_data :1;
  uint8_t   fifo_write_ch4_data :1;
  uint8_t   fifo_write_ch3_data :1;
  uint8_t   fifo_write_ch2_data :1;
  uint8_t   fifo_write_ch1_data :1;
  uint8_t   fifo_write_ch0_data :1;
  uint8_t   fifo_write_status   :1;
#endif /* DRV_BYTE_ORDER */
} as7341_fifo_map_reg_t;

/**************************************************************************************************/
/************************************** FIFO_LVL Register *********************************************/
/**************************************************************************************************/
#define FIFO_LVL_REG_ADDR (0XFD)
#define FIFO_LVL_REG_RESET_VAL (0b00000000)

typedef struct
{
  uint8_t   fifo_lvl    :8;
} as7341_fifo_lvl_reg_t;

/**************************************************************************************************/
/************************************ FDATA Register **********************************************/
/**************************************************************************************************/
#define FDATA_LOWER_REG_ADDR (0xFE)
#define FDATA_UPPER_REG_ADDR (0XFF)
#define FDATA_LOWER_REG_RESET_VAL (0b00000000)
#define FDATA_UPPER_REG_RESET_VAL (0b00000000)

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*##################################### SMUX Definitions #########################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#define SMUX_MEMORY_ADDR_LOW (0x00)
#define SMUX_MEMORY_ADDR_HIGH (0x13)

typedef enum
{
  SMUX_ASSIGNMENT_DISABLE   = 0,
  SMUX_ASSIGN_PIXEL_TO_ADC0 = 1,
  SMUX_ASSIGN_PIXEL_TO_ADC1 = 2,
  SMUX_ASSIGN_PIXEL_TO_ADC2 = 3,
  SMUX_ASSIGN_PIXEL_TO_ADC3 = 4,
  SMUX_ASSIGN_PIXEL_TO_ADC4 = 5,
  SMUX_ASSIGN_PIXEL_TO_ADC5 = 6,
} as7341_smux_assignment_t;

typedef enum
{
  DISABLE,
  F1,
  F2,
  F3,
  F4,
  F5,
  F6,
  F7,
  F8,
  CLEAR,
  NIR,
  FLICKER,
  GPIO,
  EXT_INT,
  DARK
} as7341_smux_channels_t;

typedef struct
{
  as7341_smux_channels_t    adc_0_assignment;
  as7341_smux_channels_t    adc_1_assignment;
  as7341_smux_channels_t    adc_2_assignment;
  as7341_smux_channels_t    adc_3_assignment;
  as7341_smux_channels_t    adc_4_assignment;
  as7341_smux_channels_t    adc_5_assignment;
} as7341_smux_assignment;


//typedef struct
//{
//  as7341_smux_assignment_t  pixel_assignment    :3;
//  uint8_t                   unsed               :1;
//} as7341_smux_nibble_t;
//
//typedef struct
//{
//#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
//  as7341_smux_assignment_t  pixel_assignment    :3;
//  uint8_t                   reserved            :5;
//#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
//  uint8_t                   reserved            :5;
//  as7341_smux_assignment_t  pixel_assignment    :3;
//#endif /* DRV_BYTE_ORDER */
//} as7341_smux_low_t;
//
//typedef struct
//{
//#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
//  uint8_t                   reserved0           :4;
//  as7341_smux_assignment_t  pixel_assignment    :3;
//  uint8_t                   reserved1           :1;
//#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
//  uint8_t                   reserved1           :1;
//  as7341_smux_assignment_t  pixel_assignment    :3;
//  uint8_t                   reserved0           :4;
//#endif /* DRV_BYTE_ORDER */
//} as7341_smux_high_t;
//
//typedef struct
//{
//#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
//  as7341_smux_assignment_t  pixel_assignment_low    :3;
//  uint8_t                   reserved0               :1;
//  as7341_smux_assignment_t  pixel_assignment_high   :3;
//  uint8_t                   reserved1               :1;
//#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
//  uint8_t                   reserved1               :1;
//  as7341_smux_assignment_t  pixel_assignment_high   :3;
//  uint8_t                   reserved0               :1;
//  as7341_smux_assignment_t  pixel_assignment_low    :3;
//#endif /* DRV_BYTE_ORDER */
//} as7341_smux_both_t;
//
//typedef struct
//{
//#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
//  as7341_smux_high_t    f3_left;        // 0x00
//  as7341_smux_low_t     f1_left;        // 0x01
//  uint8_t               unused0;        // 0x02
//  as7341_smux_high_t    f8_left;        // 0x03
//  as7341_smux_low_t     f6_left;        // 0x04
//  as7341_smux_both_t    f2_f4_left;     // 0x05
//  as7341_smux_high_t    f5_left;        // 0x06
//  as7341_smux_low_t     f7_left;        // 0x07
//  as7341_smux_high_t    clear_left;     // 0x08
//  as7341_smux_high_t    f5_right;       // 0x09
//  as7341_smux_low_t     f7_right;       // 0x0A
//  uint8_t               unused1;        // 0x0B
//  as7341_smux_high_t    f2_right;       // 0x0C
//  as7341_smux_low_t     f4_right;       // 0x0D
//  as7341_smux_both_t    f8_f6_right;    // 0x0E
//  as7341_smux_high_t    f3_right;       // 0x0F
//  as7341_smux_both_t    f1_right_gpio;  // 0x10
//  as7341_smux_both_t    int_clear_right;// 0x11
//  as7341_smux_high_t    dark;           // 0x12
//  as7341_smux_both_t    nir_flicker;    // 0x13
//#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
//  as7341_smux_both_t    nir_flicker;    // 0x13
//  as7341_smux_high_t    dark;           // 0x12
//  as7341_smux_both_t    int_clear_right;// 0x11
//  as7341_smux_both_t    f1_right_gpio;  // 0x10
//  as7341_smux_high_t    f3_right;       // 0x0F
//  as7341_smux_both_t    f8_f6_right;    // 0x0E
//  as7341_smux_low_t     f4_right;       // 0x0D
//  as7341_smux_high_t    f2_right;       // 0x0C
//  uint8_t               unused1;        // 0x0B
//  as7341_smux_low_t     f7_right;       // 0x0A
//  as7341_smux_high_t    f5_right;       // 0x09
//  as7341_smux_high_t    clear_left;     // 0x08
//  as7341_smux_low_t     f7_left;        // 0x07
//  as7341_smux_high_t    f5_left;        // 0x06
//  as7341_smux_both_t    f2_f4_left;     // 0x05
//  as7341_smux_low_t     f6_left;        // 0x04
//  as7341_smux_high_t    f8_left;        // 0x03
//  uint8_t               unused0;        // 0x02
//  as7341_smux_low_t     f1_left;        // 0x01
//  as7341_smux_high_t    f3_left;        // 0x00
//#endif /* DRV_BYTE_ORDER */
//} as7341_smux_memory;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*################################## Function Declarations #######################################*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

int32_t as7341_register_io_functions    ( dev_ctx_t *dev_handle, dev_init_ptr init_fn,
                                          dev_deinit_ptr deinit_fn, dev_write_ptr bus_write_fn,
                                          dev_read_ptr bus_read_fn, dev_ms_delay_ptr delay,
                                          void *optional_handle );
int32_t as7341_set_integration_mode     (dev_ctx_t *dev_handle, as7341_int_mode_t mode);
int32_t as7341_config_smux              (dev_ctx_t *dev_handle, as7341_smux_assignment *smux_assignment);
int32_t as7341_power                    (dev_ctx_t *dev_handle, bool on);
int32_t as7341_smux_config              (dev_ctx_t *dev_handle, bool enable);
int32_t as7341_wait_config              (dev_ctx_t *dev_handle, bool enable);
int32_t as7341_spectral_meas_config     (dev_ctx_t *dev_handle, bool enable);



// @formatter:on
#endif /* COMPONENTS_DRIVERS_INC_AS7341_REG_H_ */

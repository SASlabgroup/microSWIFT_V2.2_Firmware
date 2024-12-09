/*
 * sbd.h
 *
 *  Created on: Sep 18, 2024
 *      Author: philbush
 */

#ifndef INC_SBD_H_
#define INC_SBD_H_

#include "time.h"
#include "NEDWaves/rtwhalf.h"
#define TYPE_99_CHAR_BUF_LEN 320
#define IRIDIUM_SBD_MAX_LENGTH 340

// @formatter:off

// Primary message type for wave dynamic measurements
typedef struct
{
            char        legacy_number_7;
            uint8_t     type;
            uint8_t     port;
  __packed  uint16_t    size;
  __packed  real16_T    Hs;
  __packed  real16_T    Tp;
  __packed  real16_T    Dp;
  __packed  real16_T    E_array[42];
  __packed  real16_T    f_min;
  __packed  real16_T    f_max;
  signed    char        a1_array[42];
  signed    char        b1_array[42];
  signed    char        a2_array[42];
  signed    char        b2_array[42];
  unsigned  char        cf_array[42];
  __packed  float       Lat;
  __packed  float       Lon;
  __packed  real16_T    mean_temp;
  __packed  real16_T    mean_salinity;
  __packed  real16_T    mean_voltage;
  __packed  uint32_t    timestamp;
  __packed  uint32_t    error_bits;
            uint8_t     checksum_a;
            uint8_t     checksum_b;
} sbd_message_type_52;

// Message definition for turbidity (OBS) measurements
typedef struct
{
            char        legacy_number_7;
            uint8_t     type;
  __packed  int32_t     start_lat;
  __packed  int32_t     start_lon;
  __packed  int32_t     end_lat;
  __packed  int32_t     end_lon;
  __packed  uint32_t    start_timestamp;
  __packed  uint32_t    end_timestamp;
  __packed  uint16_t    backscatter_avgs[17];
  __packed  uint16_t    ambient_avgs[17];
} sbd_message_type_60_element;

#define TURBIDITY_MSGS_PER_SBD (IRIDIUM_SBD_MAX_LENGTH / sizeof(sbd_message_type_60_element))

// Definition for the SBD message with multiple sample windows
typedef struct
{
  sbd_message_type_60_element elements[TURBIDITY_MSGS_PER_SBD];
  uint8_t                     checksum_a;
  uint8_t                     checksum_b;
} sbd_message_type_60;

// Message definition for light measurements
typedef struct
{
            char        legacy_number_7;
            uint8_t     type;
  __packed  int32_t     start_lat;
  __packed  int32_t     start_lon;
  __packed  int32_t     end_lat;
  __packed  int32_t     end_lon;
  __packed  uint32_t    start_timestamp;
  __packed  uint32_t    end_timestamp;
  __packed  uint16_t    max_reading_clear;
  __packed  uint16_t    min_reading_clear;
  __packed  uint16_t    avg_clear;
  __packed  uint16_t    avg_f1;
  __packed  uint16_t    avg_f2;
  __packed  uint16_t    avg_f3;
  __packed  uint16_t    avg_f4;
  __packed  uint16_t    avg_f5;
  __packed  uint16_t    avg_f6;
  __packed  uint16_t    avg_f7;
  __packed  uint16_t    avg_f8;
} sbd_message_type_61_element;

#define LIGHT_MSGS_PER_SBD (IRIDIUM_SBD_MAX_LENGTH / sizeof(sbd_message_type_61_element))

// Definition for the SBD message with multiple sample windows
typedef struct
{
  sbd_message_type_61_element elements[LIGHT_MSGS_PER_SBD];
  uint8_t                     checksum_a;
  uint8_t                     checksum_b;
} sbd_message_type_61;
// @formatter:on
#endif /* INC_SBD_H_ */

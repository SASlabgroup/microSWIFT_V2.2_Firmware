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
#define IRIDIUM_SBD_OVERHEAD_BYTES 2
#define TELEMETRY_FIELD_ERROR_CODE (0x70E2)

// @formatter:off

typedef struct
{
  uint8_t     checksum_a;
  uint8_t     checksum_b;
} iridium_checksum_t;

// Primary message type for wave dynamic measurements
typedef struct __packed
{
            char                legacy_number_7;
            uint8_t             type;
            uint8_t             port;
    uint16_t            size;
  __packed  real16_T            Hs;
  __packed  real16_T            Tp;
  __packed  real16_T            Dp;
  __packed  real16_T            E_array[42];
  __packed  real16_T            f_min;
  __packed  real16_T            f_max;
  signed    char                a1_array[42];
  signed    char                b1_array[42];
  signed    char                a2_array[42];
  signed    char                b2_array[42];
  unsigned  char                cf_array[42];
  __packed  float               Lat;
  __packed  float               Lon;
  __packed  real16_T            mean_temp;
  __packed  real16_T            mean_salinity;
  __packed  real16_T            mean_voltage;
  __packed  float               timestamp;
  __packed  uint32_t            error_bits;
            iridium_checksum_t  checksum;
} sbd_message_type_52;

// Message definition for turbidity (OBS) measurements
typedef struct __packed
{
  int32_t     start_lat;
  int32_t     start_lon;
  int32_t     end_lat;
  int32_t     end_lon;
  uint32_t    start_timestamp;
  uint32_t    end_timestamp;
  uint16_t    backscatter_avgs[17];
  uint16_t    ambient_avgs[17];
} sbd_message_type_53_element;

#define TURBIDITY_MSGS_PER_SBD ((IRIDIUM_SBD_MAX_LENGTH - IRIDIUM_SBD_OVERHEAD_BYTES) / sizeof(sbd_message_type_53_element))

// Definition for the SBD message with multiple sample windows
typedef struct __packed
{
  char                        legacy_number_7;
  uint8_t                     type;
  uint8_t                     port;
  uint16_t                    size;
  sbd_message_type_53_element elements[TURBIDITY_MSGS_PER_SBD];
  iridium_checksum_t          checksum;
} sbd_message_type_53;

// Message definition for light measurements
typedef struct __packed
{
  int32_t     start_lat;
  int32_t     start_lon;
  int32_t     end_lat;
  int32_t     end_lon;
  uint32_t    start_timestamp;
  uint32_t    end_timestamp;
  uint16_t    max_reading_clear;
  uint16_t    min_reading_clear;
  uint16_t    avg_clear;
  uint16_t    avg_f1;
  uint16_t    avg_f2;
  uint16_t    avg_f3;
  uint16_t    avg_f4;
  uint16_t    avg_f5;
  uint16_t    avg_f6;
  uint16_t    avg_f7;
  uint16_t    avg_f8;
  uint16_t    avg_dark;
  uint16_t    avg_nir;
} sbd_message_type_54_element;

#define LIGHT_MSGS_PER_SBD ((IRIDIUM_SBD_MAX_LENGTH - IRIDIUM_SBD_OVERHEAD_BYTES) / sizeof(sbd_message_type_54_element))

// Definition for the SBD message with multiple sample windows
typedef struct __packed
{
  char                        legacy_number_7;
  uint8_t                     type;
  uint8_t                     port;
  uint16_t                    size;
  sbd_message_type_54_element elements[LIGHT_MSGS_PER_SBD];
  iridium_checksum_t          checksum;
} sbd_message_type_54;
// @formatter:on
#endif /* INC_SBD_H_ */

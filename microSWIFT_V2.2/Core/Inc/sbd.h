/*
 * sbd.h
 *
 *  Created on: Sep 18, 2024
 *      Author: philbush
 */
// clang-format off

#ifndef INC_SBD_H_
#define INC_SBD_H_

#include "time.h"
#include "NEDWaves/rtwhalf.h"
#define TYPE_99_CHAR_BUF_LEN 320
#define IRIDIUM_SBD_MAX_LENGTH 340
#define TELEMETRY_FIELD_ERROR_CODE (0x70E2)

// @formatter:off

typedef struct
{
  uint8_t     checksum_a;
  uint8_t     checksum_b;
} iridium_checksum_t;
#define IRIDIUM_CHECKSUM_LENGTH sizeof(iridium_checksum_t)

// Primary message type for wave dynamic measurements
typedef struct __packed
{
  char                legacy_number_7;
  uint8_t             type;
  uint8_t             port;
  uint16_t            size;
  real16_T            Hs;
  real16_T            Tp;
  real16_T            Dp;
  real16_T            E_array[42];
  real16_T            f_min;
  real16_T            f_max;
  signed    char      a1_array[42];
  signed    char      b1_array[42];
  signed    char      a2_array[42];
  signed    char      b2_array[42];
  unsigned  char      cf_array[42];
  float               Lat;
  float               Lon;
  real16_T            mean_temp;
  real16_T            mean_salinity;
  real16_T            mean_voltage;
  float               timestamp;
  uint32_t            error_bits;
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

#define TURBIDITY_MSGS_PER_SBD ((IRIDIUM_SBD_MAX_LENGTH - IRIDIUM_CHECKSUM_LENGTH) / sizeof(sbd_message_type_53_element))

// Definition for the SBD message with multiple sample windows
typedef struct __packed
{
  char                        legacy_number_7;
  uint8_t                     type;
  uint8_t                     port;
  uint16_t                    size;
  uint16_t                    serial_number;
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

#define LIGHT_MSGS_PER_SBD ((IRIDIUM_SBD_MAX_LENGTH - IRIDIUM_CHECKSUM_LENGTH) / sizeof(sbd_message_type_54_element))

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

// Definition for accelerometer waves message; based heavily on type_52 for the
// waves component.
typedef struct __packed
{
  char legacy_number_7;  // byte 0
  uint8_t type;  // byte 1

  // I'm hoping we can omit this, otherwise will have to find 3 bytes elsewhere
  // uint8_t             port;
  // uint16_t            size;

  uint32_t timestamp; // byte 2-5

  float latitude;  // byte 6-9
  float longitude;  // byte 10-13

  // bit 0-1: sensitivity (+/- 2, 4, 8g) == {00, 01, 10}
  // bit 2: == 0 for low-rate spectra; == 1 for high-rate
  // bit 3: ==0 for background/duty-cycle; == 1 for wake-on-shake
  // bit 4-7: threshold for wake-on-shake; TODO: define how this translates to counts
  uint8_t config;  // byte 14
  // bit 0-3: count of wake-on-shake events since last duty cycle
  // bit 4-7: TBD
  uint8_t status;  // byte 15

  uint16_t min_x_accel;  // byte 16-21
  uint16_t max_x_accel;
  uint16_t mean_x_accel;

  uint16_t min_y_accel;  // byte 22-27
  uint16_t max_y_accel;
  uint16_t mean_y_accel;

  uint16_t min_z_accel;  // byte 28-33
  uint16_t max_z_accel;
  uint16_t mean_z_accel;

  // at 34 bytes here...

  // the Waves component is 204 bytes
  real16_T Hs;
  real16_T Tp;
  real16_T Dp;
  real16_T E_array[42];
  real16_T f_min;
  real16_T f_max;
  signed char a1_array[42];
  signed char b1_array[42];
  signed char a2_array[42];
  signed char b2_array[42];
  unsigned char cf_array[42];

  // 2 bytes for the checksum; this MUST be the final element in the struct,
  // since the iridium thread calculates the checksum and calls memcpy on
  // payload[sizeof(sbd_message_type_55) - IRIDIUM_CHECKSUM_LENGTH]
  iridium_checksum_t  checksum;
} sbd_message_type_55;

typedef struct
{
  char                  message_body[TYPE_99_CHAR_BUF_LEN];
  float                 latitude;
  float                 longitude;
  uint32_t              timestamp;
  iridium_checksum_t    checksum;
} sbd_message_type_99;
// @formatter:on
#endif /* INC_SBD_H_ */

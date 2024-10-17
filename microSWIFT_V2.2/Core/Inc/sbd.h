/*
 * sbd.h
 *
 *  Created on: Sep 18, 2024
 *      Author: philbush
 */

#ifndef INC_SBD_H_
#define INC_SBD_H_

// @formatter:off
typedef struct sbd_message_type_52
{
            char     legacy_number_7;
            uint8_t  type;
            uint8_t  port;
  __packed  uint16_t size;
  __packed  real16_T Hs;
  __packed  real16_T Tp;
  __packed  real16_T Dp;
  __packed  real16_T E_array[42];
  __packed  real16_T f_min;
  __packed  real16_T f_max;
  signed    char     a1_array[42];
  signed    char     b1_array[42];
  signed    char     a2_array[42];
  signed    char     b2_array[42];
  unsigned  char     cf_array[42];
  __packed  float    Lat;
  __packed  float    Lon;
  __packed  real16_T mean_temp;
  __packed  real16_T mean_salinity;
  __packed  real16_T mean_voltage;
  __packed  uint32_t timestamp;
            uint8_t  checksum_a;
            uint8_t  checksum_b;
} sbd_message_type_52;

typedef struct
{
            uint8_t   number_99;
            char      char_buf[320];
  __packed  float     lat;
  __packed  float     lon;
  __packed  time_t    timetamp;
} sbd_message_type_99;

// @formatter:on
#endif /* INC_SBD_H_ */

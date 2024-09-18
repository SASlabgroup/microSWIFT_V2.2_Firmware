/*
 * Iridium.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_IRIDIUM_H_
#define SRC_IRIDIUM_H_

#include "tx_api.h"
#include "NEDWaves/rtwhalf.h"
#include "stdbool.h"
#include "time.h"
#include "configuration.h"
#include "generic_uart_driver.h"
#include "gpio.h"

// @formatter:off

// Return codes
typedef enum iridium_error_code
{
  IRIDIUM_SUCCESS = 0,
  IRIDIUM_UNKNOWN_ERROR = -1,
  IRIDIUM_UART_ERROR = -2,
  IRIDIUM_TRANSMIT_ERROR = -3,
  IRIDIUM_COMMAND_RESPONSE_ERROR = -4,
  IRIDIUM_SELF_TEST_FAILED = -5,
  IRIDIUM_RECEIVE_ERROR = -6,
  IRIDIUM_FLASH_STORAGE_ERROR = -7,
  IRIDIUM_STORAGE_QUEUE_FULL = -8,
  IRIDIUM_STORAGE_QUEUE_EMPTY = -9,
  IRIDIUM_TIMER_ERROR = -10,
  IRIDIUM_TRANSMIT_TIMEOUT = -11,
  IRIDIUM_TRANSMIT_UNSUCCESSFUL = -12
} iridium_error_code_t;

// Macros
#define IRIDIUM_INITIAL_CAP_CHARGE_TIME TX_TIMER_TICKS_PER_SECOND * 30
#define IRIDIUM_TOP_UP_CAP_CHARGE_TIME TX_TIMER_TICKS_PER_SECOND * 10
#define MAX_RETRIES 10
#define ACK_MESSAGE_SIZE 9
#define DISABLE_FLOW_CTRL_SIZE 12
#define ENABLE_RI_SIZE 18
#define STORE_CONFIG_SIZE 12
#define SELECT_PWR_UP_SIZE 12
#define SBDWB_READY_RESPONSE_SIZE 22
#define SBDWB_LOAD_RESPONSE_SIZE 11
#define SBDWB_RESPONSE_CODE_INDEX 2
#define SBDI_RESPONSE_CODE_INDEX 16
#define SBDIX_RESPONSE_CODE_INDEX 18
#define SBDI_RESPONSE_SIZE 19
#define SBDIX_RESPONSE_SIZE 30
#define SBDD_RESPONSE_SIZE 20
#define SBDIX_WAIT_TIME ONE_SECOND * 13
#define IRIDIUM_MESSAGE_PAYLOAD_SIZE 327
#define IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE 333
#define ERROR_MESSAGE_TYPE 99
#define IRIDIUM_CHECKSUM_LENGTH 2
#define IRIDIUM_MAX_RESPONSE_SIZE 64
#define IRIDIUM_FET_PIN 0
#define IRIDIUM_NETAV_PIN 0
#define IRIDIUM_RI_PIN 0
#define IRIDIUM_ONOFF_PIN 0
#define IRIDIUM_DEFAULT_BAUD_RATE 19200
#define CHECKSUM_FIRST_BYTE_INDEX 327
#define CHECKSUM_SECOND_BYTE_INDEX 328
#define ONE_SECOND 1000
#define ASCII_ZERO 48
#define ASCII_FIVE 53
#define IRIDIUM_MAX_TRANSMIT_PERIOD 6 - 1
#define ERROR_MESSAGE_MAX_LENGTH 320
#define IRIDIUM_TIMER_INSTANCE TIM17
#define IRIDIUM_UART_INSTANCE UART5
#define IRIDIUM_LL_TX_DMA_HANDLE LL_DMA_CHANNEL_2
#define IRIDIUM_LL_RX_DMA_HANDLE LL_DMA_CHANNEL_3
#define SECONDS_IN_MIN 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000 // 365 day year, not accounting for 1/4 leap days
#define SECONDS_1970_TO_2000 946684800
#define EPOCH_YEAR 1970
#define CURRENT_CENTURY 2000
// Magic number used to indicate the SBD message queue has been initialized
#define SBD_QUEUE_MAGIC_NUMBER 0x101340

typedef struct sbd_message_type_52
{
  char              legacy_number_7;
  uint8_t           type;
  uint8_t           port;
  __packed uint16_t size;
  __packed real16_T Hs;
  __packed real16_T Tp;
  __packed real16_T Dp;
  __packed real16_T E_array[42];
  __packed real16_T f_min;
  __packed real16_T f_max;
  signed char       a1_array[42];
  signed char       b1_array[42];
  signed char       a2_array[42];
  signed char       b2_array[42];
  unsigned char     cf_array[42];
  __packed float    Lat;
  __packed float    Lon;
  __packed real16_T mean_temp;
  __packed real16_T mean_salinity;
  __packed real16_T mean_voltage;
  __packed float    timestamp;
  uint8_t           checksum_a;
  uint8_t           checksum_b;
} sbd_message_type_52;

typedef struct Iridium
{
  // Our global configuration struct
  microSWIFT_configuration *global_config;
  // UART driver
  generic_uart_driver       uart_driver;
  // DMA handles for the Iridium UART port
  DMA_HandleTypeDef         *uart_tx_dma_handle;
  DMA_HandleTypeDef         *uart_rx_dma_handle;
  // Pointer to hardware timer handle
  TX_TIMER                  *timer;
  // Event flags
  TX_EVENT_FLAGS_GROUP      *error_flags;
  // pointer to the message array
  sbd_message_type_52       *current_message;
  // Error message payload buffer
  uint8_t                   error_message_buffer[IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH + 64];
  // UART response buffer
  uint8_t                   response_buffer[IRIDIUM_MAX_RESPONSE_SIZE];

  gpio_pin_struct           bus_5v_fet;
  gpio_pin_struct           pwr_pin;
  gpio_pin_struct           sleep_pin;


  bool                      timer_timeout;

  iridium_error_code_t      (*config) ( void );
  iridium_error_code_t      (*self_test) ( void );
  iridium_error_code_t      (*transmit_message) ( void );
  iridium_error_code_t      (*transmit_error_message) ( char *error_message );
  iridium_error_code_t      (*start_timer) ( uint16_t timeout_in_minutes );
  iridium_error_code_t      (*stop_timer) ( void );
  float                     (*get_timestamp) ( void );
  void                      (*sleep) ( void );
  void                      (*wake) ( void );
  void                      (*on) ( void );
  void                      (*off) ( void );
} Iridium;

/* Function declarations */
void iridium_init ( Iridium *struct_ptr, microSWIFT_configuration *global_config,
                    UART_HandleTypeDef *iridium_uart_handle, DMA_HandleTypeDef *uart_tx_dma_handle,
                    DMA_HandleTypeDef *uart_rx_dma_handle, TX_TIMER *timer,
                    TX_EVENT_FLAGS_GROUP *error_flags, sbd_message_type_52 *current_message );
void iridium_deinit ( void );
void iridium_timer_expired ( ULONG expiration_input );
bool iridium_get_timeout_status ( void );


// @formatter:on
#endif /* SRC_IRIDIUM_H_ */

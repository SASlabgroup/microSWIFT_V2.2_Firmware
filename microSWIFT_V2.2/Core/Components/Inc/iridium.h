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
#include "sbd.h"
#include "microSWIFT_return_codes.h"

// @formatter:off

// Macros
#define IRIDIUM_INITIAL_CAP_CHARGE_TIME (TX_TIMER_TICKS_PER_SECOND * 15U)
#define IRIDIUM_TOP_UP_CAP_CHARGE_TIME (TX_TIMER_TICKS_PER_SECOND * 5U)
#define IRIDIUM_MAX_UART_TX_TICKS (TX_TIMER_TICKS_PER_SECOND)
#define IRIDIUM_MAX_UART_RX_TICKS_NO_TX (TX_TIMER_TICKS_PER_SECOND)
#define IRIDIUM_MAX_UART_RX_TICKS_TX (TX_TIMER_TICKS_PER_SECOND * 45U)
#define MODEM_SLEEP_TIME (TX_TIMER_TICKS_PER_SECOND * 23U)
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
#define IRIDIUM_CHECKSUM_LENGTH sizeof(iridium_checksum_t)
#define IRIDIUM_MAX_RESPONSE_SIZE 64
#define IRIDIUM_DEFAULT_BAUD_RATE 19200
#define CHECKSUM_FIRST_BYTE_INDEX 327
#define CHECKSUM_SECOND_BYTE_INDEX 328
#define ONE_SECOND 1000
#define ASCII_ZERO 48
#define ASCII_FIVE 53
#define ERROR_MESSAGE_MAX_LENGTH 320

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
  // Error message payload buffer
  uint8_t                   error_message_buffer[IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH + 64];
  // UART response buffer
  uint8_t                   response_buffer[IRIDIUM_MAX_RESPONSE_SIZE];

  gpio_pin_struct           bus_5v_fet;
  gpio_pin_struct           pwr_pin;
  gpio_pin_struct           sleep_pin;


  bool                      timer_timeout;

  uSWIFT_return_code_t      (*config) ( void );
  uSWIFT_return_code_t      (*self_test) ( void );
  uSWIFT_return_code_t      (*start_timer) ( uint16_t timeout_in_minutes );
  uSWIFT_return_code_t      (*stop_timer) ( void );
  uSWIFT_return_code_t      (*transmit_message) ( uint8_t *msg, uint32_t msg_size );
  void                      (*charge_caps) ( uint32_t caps_charge_time_ticks );
  void                      (*sleep) ( void );
  void                      (*wake) ( void );
  void                      (*on) ( void );
  void                      (*off) ( void );
} Iridium;

/* Function declarations */
void iridium_init ( Iridium *struct_ptr, microSWIFT_configuration *global_config,
                    UART_HandleTypeDef *iridium_uart_handle, TX_SEMAPHORE *uart_sema,
                    DMA_HandleTypeDef *uart_tx_dma_handle, DMA_HandleTypeDef *uart_rx_dma_handle,
                    TX_TIMER *timer, TX_EVENT_FLAGS_GROUP *error_flags );
void iridium_deinit ( void );
void iridium_timer_expired ( ULONG expiration_input );
bool iridium_get_timeout_status ( void );


// @formatter:on
#endif /* SRC_IRIDIUM_H_ */

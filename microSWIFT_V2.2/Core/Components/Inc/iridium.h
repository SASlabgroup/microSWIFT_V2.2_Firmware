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
#define IRIDIUM_INITIAL_CAP_CHARGE_TIME (TX_TIMER_TICKS_PER_SECOND * 30U)
#define IRIDIUM_TOP_UP_CAP_CHARGE_TIME (TX_TIMER_TICKS_PER_SECOND * 10U)
#define IRIDIUM_MAX_UART_TX_TICKS (TX_TIMER_TICKS_PER_SECOND)
#define IRIDIUM_MAX_UART_RX_TICKS_NO_TX (TX_TIMER_TICKS_PER_SECOND  * 5U)
#define IRIDIUM_MAX_UART_RX_TICKS_TX (TX_TIMER_TICKS_PER_SECOND * 45U)
#define MODEM_SLEEP_TIME (TX_TIMER_TICKS_PER_SECOND * 23U)
#define ACK_MESSAGE_SIZE 9U
#define DISABLE_FLOW_CTRL_SIZE 12U
#define ENABLE_RI_SIZE 18U
#define STORE_CONFIG_SIZE 12U
#define SELECT_PWR_UP_SIZE 12U
#define FLUSH_TO_EPPROM_SIZE 11U
#define SBDWB_READY_RESPONSE_SIZE 22U
#define SBDWB_LOAD_RESPONSE_SIZE 11U
#define SBDWB_RESPONSE_CODE_INDEX 2U
#define SBDI_RESPONSE_CODE_INDEX 16U
#define SBDIX_RESPONSE_CODE_INDEX 18U
#define SBDI_RESPONSE_SIZE 19U
#warning "change this back to 30"
#define SBDIX_RESPONSE_SIZE 40U
#define SBDD_RESPONSE_SIZE 20U
#define SBDIX_WAIT_TIME ONE_SECOND * 13U
#define ERROR_MESSAGE_TYPE 99U
#define IRIDIUM_CHECKSUM_LENGTH sizeof(iridium_checksum_t)
#define IRIDIUM_MAX_RESPONSE_SIZE 64U
#define IRIDIUM_DEFAULT_BAUD_RATE 19200U
#define CHECKSUM_FIRST_BYTE_INDEX 327U
#define CHECKSUM_SECOND_BYTE_INDEX 328U
#define ASCII_ZERO 48U
#define ASCII_FIVE 53U

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
  // UART response buffer
  uint8_t                   response_buffer[IRIDIUM_MAX_RESPONSE_SIZE];

  gpio_pin_struct           bus_5v_fet;
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
  void                      (*cycle_power) ( void );
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

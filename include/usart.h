/**
 * @file usart.h
 * @brief USART3 USB Serial Communication Driver
 */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "protocol.h"

/* Global USART3 Handle */
extern UART_HandleTypeDef huart3;

/**
 * @brief Initialize USART3 peripheral on PD8 (TX) / PD9 (RX) at 115200 baud.
 * Also starts the interrupt-driven command receiver (see uart_protocol_rx_isr).
 */
void MX_USART3_UART_Init(void);

/**
 * @brief Frame and blocking-send a telemetry packet.
 */
void uart_send_telemetry(const telemetry_packet_t *pkt);

/**
 * @brief Latest fully-received, checksum-valid command packet. Only the
 * fields after 'type' are meaningful to callers.
 */
extern volatile command_packet_t g_last_command;

/**
 * @brief Tick (HAL_GetTick()) of the last valid command packet received.
 */
extern volatile uint32_t g_last_command_tick;

/**
 * @brief 1 if no valid command has arrived within timeout_ms (host link
 * lost/stale) - caller should fail safe (stop motors) in that case.
 */
uint8_t uart_command_is_stale(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */

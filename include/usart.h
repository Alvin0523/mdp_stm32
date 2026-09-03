/**
 * @file usart.h
 * @brief USART1 (debug console) + USART3 (binary protocol) Serial Drivers
 *
 * Two independent physical USB-C ports on this board (both via the onboard
 * CH9102F), kept on separate UARTs so a human-readable text console and the
 * RPi's binary telemetry/command wire never share bytes on the same port:
 *   - USART1 (PA9/PA10, USB port 1): debug console. printf() is retargeted
 *     here - plug this port in and run `pio device monitor` for clean,
 *     human-readable log text only.
 *   - USART3 (PD8/PD9, USB port 3): binary command/telemetry link to the
 *     RPi (see protocol.h). Never printf() anything on this port - the
 *     RPi-side framer (mdp_ros/src/mdp_hardware_bridge) expects every byte
 *     to be part of the framed binary protocol.
 */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "protocol.h"

/* Global USART1 (debug console) and USART3 (binary protocol) Handles */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/**
 * @brief Initialize USART1 peripheral on PA9 (TX) / PA10 (RX) at 115200
 * baud. TX-only in practice: printf() is retargeted here via _write().
 */
void MX_USART1_UART_Init(void);

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

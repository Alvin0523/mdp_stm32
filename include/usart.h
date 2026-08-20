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

/* Global USART3 Handle */
extern UART_HandleTypeDef huart3;

/**
 * @brief Initialize USART3 peripheral on PD8 (TX) / PD9 (RX) at 115200 baud
 */
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */

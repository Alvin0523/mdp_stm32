/**
 * @file usart.c
 * @brief USART1 Debug Console + USART3 Binary Protocol/printf() Retargeting
 *
 * HOW THIS WORKS:
 * 1. Hardware Pinout:
 *    On WHEELTEC C30D (Revision 2.1), the CH9102F USB-to-UART converter chip
 *    exposes two independent USB-C ports, each wired to its own USART:
 *      - USB Port 1 -> USART1: PA9 = TX1, PA10 = RX1 (debug console)
 *      - USB Port 3 -> USART3: PD8 = TX3, PD9 = RX3 (binary protocol to RPi)
 *
 * 2. Clock Gating:
 *    We must enable the GPIO port clock and the USART's own clock before
 *    configuring registers (GPIOA/USART1 and GPIOD/USART3 respectively).
 *
 * 3. Alternate Function (AF7):
 *    All four pins are GPIO_MODE_AF_PP with Alternate Function 7 (AF7_USARTx).
 *
 * 4. Retargeting printf():
 *    Overriding GCC syscall `_write()` redirects standard C library `printf()`
 *    through HAL_UART_Transmit() on USART1 (the debug console) - deliberately
 *    NOT USART3, so debug text never gets interleaved with the binary
 *    telemetry/command frames the RPi's framer (mdp_hardware_bridge) expects
 *    on that wire.
 */

#include "usart.h"
#include <string.h>
#include <stddef.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

volatile command_packet_t g_last_command = {0};
volatile uint32_t g_last_command_tick = 0;

/* Byte-at-a-time RX framing state machine, driven from HAL_UART_RxCpltCallback. */
typedef enum {
    RX_WAIT_SYNC0,
    RX_WAIT_SYNC1,
    RX_WAIT_TYPE,
    RX_PAYLOAD,
} rx_state_t;

static rx_state_t s_rx_state = RX_WAIT_SYNC0;
static uint8_t s_rx_byte;
static uint8_t s_rx_buf[sizeof(command_packet_t)];
static uint8_t s_rx_index;

static uint8_t protocol_checksum(const uint8_t *bytes, uint32_t len)
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < len; i++) {
        checksum ^= bytes[i];
    }
    return checksum;
}

void MX_USART1_UART_Init(void)
{
    /* Step 1: Enable Peripheral Clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Step 2: Configure GPIO Pins PA9 (TX) and PA10 (RX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Step 3: Configure USART1 Settings (115200 Baud, 8-N-1) */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void MX_USART3_UART_Init(void)
{
    /* Step 1: Enable Peripheral Clocks */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Step 2: Configure GPIO Pins PD8 (TX) and PD9 (RX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           /* Push-Pull Alternate Function */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* High Slew Rate for 115200 Baud */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;      /* Connect Pin to USART3 Controller */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Step 3: Configure USART3 Settings (115200 Baud, 8-N-1) */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;               /* Enable both Transmit & Receive */
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    /* Step 4: Enable USART3 RX interrupt and start byte-at-a-time reception
     * for the command packet framing state machine. */
    /* Below the motor PID ISR (priority 1, motor.c), above the user button
     * (priority 3, button.c) - see docs/stm32/control_loop.md's interrupt
     * priority table for the full rationale. */
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    switch (s_rx_state) {
        case RX_WAIT_SYNC0:
            if (s_rx_byte == PROTOCOL_SYNC0) {
                s_rx_state = RX_WAIT_SYNC1;
            }
            break;

        case RX_WAIT_SYNC1:
            s_rx_state = (s_rx_byte == PROTOCOL_SYNC1) ? RX_WAIT_TYPE : RX_WAIT_SYNC0;
            break;

        case RX_WAIT_TYPE:
            if (s_rx_byte == PROTOCOL_TYPE_COMMAND) {
                s_rx_buf[0] = s_rx_byte;
                s_rx_index = 1;
                s_rx_state = RX_PAYLOAD;
            } else {
                s_rx_state = RX_WAIT_SYNC0;
            }
            break;

        case RX_PAYLOAD:
            s_rx_buf[s_rx_index++] = s_rx_byte;
            if (s_rx_index >= sizeof(command_packet_t) - 2) { /* minus 2 sync bytes, already consumed */
                uint8_t expected_checksum = s_rx_buf[s_rx_index - 1];
                uint8_t computed_checksum = protocol_checksum(s_rx_buf, s_rx_index - 1);
                if (computed_checksum == expected_checksum) {
                    command_packet_t cmd;
                    cmd.sync0 = PROTOCOL_SYNC0;
                    cmd.sync1 = PROTOCOL_SYNC1;
                    memcpy(&cmd.type, s_rx_buf, s_rx_index);
                    g_last_command = cmd;
                    g_last_command_tick = HAL_GetTick();
                }
                s_rx_state = RX_WAIT_SYNC0;
            }
            break;
    }

    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

uint8_t uart_command_is_stale(uint32_t timeout_ms)
{
    if (g_last_command_tick == 0) {
        return 1; /* No command ever received */
    }
    return (HAL_GetTick() - g_last_command_tick) > timeout_ms ? 1 : 0;
}

/* Must be static, not a local: HAL_UART_Transmit_IT() returns immediately
 * and the ISR shifts the bytes out in the background, so a stack-local
 * buffer would go out of scope mid-transfer. */
static telemetry_packet_t s_tx_telemetry;

void uart_send_telemetry(const telemetry_packet_t *pkt)
{
    s_tx_telemetry = *pkt;
    s_tx_telemetry.sync0 = PROTOCOL_SYNC0;
    s_tx_telemetry.sync1 = PROTOCOL_SYNC1;
    s_tx_telemetry.type = PROTOCOL_TYPE_TELEMETRY;
    s_tx_telemetry.checksum = protocol_checksum(
        (const uint8_t *)&s_tx_telemetry.type,
        sizeof(s_tx_telemetry) - offsetof(telemetry_packet_t, type) - 1);

    /* Interrupt-driven, not blocking: at the fast tier's 100Hz/10ms period
     * (main.c), blocking for the ~4.7ms this 54-byte packet takes to shift
     * out at 115200 baud would eat ~47% of the whole budget every cycle.
     * HAL_UART_Transmit_IT() kicks off the transfer and returns
     * immediately; the same USART3_IRQHandler already servicing RX finishes
     * it in the background. If the previous packet is still transmitting
     * (HAL_BUSY), this one is simply dropped - telemetry is a continuous
     * stream, not request/response, so losing one 10ms sample occasionally
     * is harmless and strictly better than blocking the fast tier on it. */
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)&s_tx_telemetry, sizeof(s_tx_telemetry));
}

/**
 * @brief Retarget standard C library _write() to USART1 (debug console)
 * @param file System file handle (unused for embedded stdout)
 * @param ptr Pointer to output text buffer
 * @param len Byte length of output text buffer
 * @return Number of written bytes
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

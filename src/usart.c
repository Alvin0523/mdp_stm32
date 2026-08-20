/**
 * @file usart.c
 * @brief USART3 USB Serial Communications & printf() Retargeting
 *
 * HOW THIS WORKS:
 * 1. Hardware Pinout:
 *    On WHEELTEC C30D (Revision 2.1), the CH9102F USB-to-UART converter chip
 *    is hardwired to USART3:
 *      - PD8 = USART3_TX (Transmit output from STM32 -> USB)
 *      - PD9 = USART3_RX (Receive input from USB -> STM32)
 *
 * 2. Clock Gating:
 *    We must enable GPIOD clock (__HAL_RCC_GPIOD_CLK_ENABLE) and USART3 clock
 *    (__HAL_RCC_USART3_CLK_ENABLE) before configuring registers.
 *
 * 3. Alternate Function (AF7):
 *    PD8 and PD9 pins are set to GPIO_MODE_AF_PP with Alternate Function 7 (AF7_USART3).
 *
 * 4. Retargeting printf():
 *    Overriding GCC syscall `_write()` redirects standard C library `printf()`
 *    directly through HAL_UART_Transmit() over USB serial.
 */

#include "usart.h"
#include <string.h>
#include <stddef.h>

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
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
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

void uart_send_telemetry(const telemetry_packet_t *pkt)
{
    telemetry_packet_t out = *pkt;
    out.sync0 = PROTOCOL_SYNC0;
    out.sync1 = PROTOCOL_SYNC1;
    out.type = PROTOCOL_TYPE_TELEMETRY;
    out.checksum = protocol_checksum((const uint8_t *)&out.type, sizeof(out) - offsetof(telemetry_packet_t, type) - 1);
    HAL_UART_Transmit(&huart3, (uint8_t *)&out, sizeof(out), HAL_MAX_DELAY);
}

/**
 * @brief Retarget standard C library _write() to USART3
 * @param file System file handle (unused for embedded stdout)
 * @param ptr Pointer to output text buffer
 * @param len Byte length of output text buffer
 * @return Number of written bytes
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

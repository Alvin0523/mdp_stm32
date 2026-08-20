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

UART_HandleTypeDef huart3;

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

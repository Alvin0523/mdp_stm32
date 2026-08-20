/**
 * @file motor.c
 * @brief AT8236 Dual H-Bridge Motor Driver Hardware Initialization
 *
 * HOW THIS WORKS:
 * 1. Hardware Motor Pin Mapping (WHEELTEC C30D Board):
 *    - Rear Left Motor (A):  PB8 (TIM10_CH1) & PB9 (TIM11_CH1)
 *    - Rear Right Motor (B): PE5 (TIM9_CH1)  & PE6 (TIM9_CH2)
 *
 * 2. PWM Drive Principle:
 *    - AT8236 motor driver accepts 2 PWM inputs per channel (IN1 / IN2).
 *    - PWM Duty Cycle (0-100%) controls motor speed via pulse-width modulation.
 *    - Direction:
 *      * IN1 = PWM, IN2 = 0: Forward
 *      * IN1 = 0, IN2 = PWM: Reverse
 *      * IN1 = 0, IN2 = 0: Coast / Stop
 */

#include "motor.h"

int motor_init(void)
{
    /* Step 1: Enable GPIO Clocks for Port B and Port E */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Step 2: Configure Motor A (Rear Left) Pins PB8 & PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            /* Alternate Function Push-Pull */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;       /* Connect PB8/PB9 to Timer AF3 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Step 3: Configure Motor B (Rear Right) Pins PE5 & PE6 */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;        /* Connect PE5/PE6 to Timer AF3 */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    return 0;
}

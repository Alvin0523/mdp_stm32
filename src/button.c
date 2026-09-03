/**
 * @file button.c
 * @brief User Button PE0 EXTI Interrupt Handler
 *
 * HOW THIS WORKS:
 * 1. PE0 Pin Setup:
 *    PE0 is connected to the onboard USER button on WHEELTEC C30D board.
 *    Configured as GPIO_MODE_IT_FALLING with Pull-Up.
 *
 * 2. EXTI0 Interrupt:
 *    When PE0 is pressed (pulled to GND), the hardware generates EXTI Line 0 Interrupt.
 *    EXTI0_IRQHandler() only raises a flag here - the actual self-test
 *    sequence runs from the main loop, since it blocks on HAL_Delay/motor
 *    control and must not run inside an ISR.
 */

#include "button.h"

static uint32_t s_last_button_tick = 0;
static volatile uint8_t s_selftest_requested = 0;

uint8_t button_consume_selftest_request(void)
{
    if (s_selftest_requested) {
        s_selftest_requested = 0;
        return 1;
    }
    return 0;
}

void button_init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Enable EXTI Line 0 Interrupt in NVIC. Lowest priority (highest
     * number) of this project's app-level interrupts - a human button
     * press has no hard real-time deadline, unlike the motor PID (priority
     * 1) or command reception (priority 2) - see docs/stm32/control_loop.md. */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief EXTI Line 0 Interrupt Handler for PE0 User Button
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief EXTI Falling Edge Callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        /* Debounce button press (200ms cooldown) */
        uint32_t now = HAL_GetTick();
        if (now - s_last_button_tick > 200) {
            s_last_button_tick = now;
            s_selftest_requested = 1; /* run self-test from the main loop */
        }
    }
}
